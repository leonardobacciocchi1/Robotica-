/**
 * @file centralized_node.cpp
 * @brief Implementazione del nodo ROS2 per la localizzazione cooperativa centralizzata
 *
 * Questo file implementa un nodo ROS2 che gestisce un filtro di Kalman esteso (EKF)
 * centralizzato per stimare la posizione di più robot contemporaneamente.
 * Il nodo riceve dati di odometria e misurazioni dai robot e applica l'algoritmo EKF
 * per mantenere una stima aggiornata delle posizioni di tutti i robot.
 *
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ekf_centralized.h"            
#include "data_loader.h"                
#include "groundtruth_loader.h"      
#include <memory>                   
#include <vector>                       
#include <set>                          

using std::placeholders::_1;

/**
 * Questa classe eredita da rclcpp::Node e implementa un sistema di localizzazione
 * cooperativa centralizzato. Il nodo:
 * - Riceve dati di odometria dai robot
 * - Riceve misurazioni landmark e robot-robot
 * - Applica l'algoritmo EKF per stimare le posizioni
 * - Pubblica periodicamente lo stato stimato
 */
class CentralizedEkfNode : public rclcpp::Node {
public:
    /**
     * Inizializza il nodo, carica i parametri, configura i subscriber,
     * inizializza l'EKF e carica i landmark dal dataset.
     */
    CentralizedEkfNode() : Node("centralized_ekf_node"),
                          last_time_(0.0), has_time_(false) {
        
        // Parametri
        declare_parameter("num_robots", 2);
        declare_parameter("dataset_path", std::string("MRCLAM1/MRCLAM_Dataset1/"));
        declare_parameter("print_interval", 5.0); 
        declare_parameter("use_groundtruth_init", true); 
        
        int num_robots;
        std::string dataset_path;
        bool use_groundtruth_init;
        
        get_parameter("num_robots", num_robots);
        get_parameter("dataset_path", dataset_path);
        get_parameter("print_interval", print_interval_);
        get_parameter("use_groundtruth_init", use_groundtruth_init);
        
        num_robots_ = num_robots;
        
	// Inizializza EKF con i groundtruth o default state
        Eigen::VectorXd initial_state;
        if (use_groundtruth_init) {
            RCLCPP_INFO(get_logger(), "Caricamento dello stato iniziale a partire dai groundtruth...");
            initial_state = getInitialStateFromGroundtruth(dataset_path, num_robots);
        } else {
            RCLCPP_INFO(get_logger(), "Utilizzo dello stato iniziale di default...");
            initial_state = Eigen::VectorXd::Zero(num_robots * 3);
        }
        
        ekf_ = std::make_unique<EkfCentralized>(num_robots, initial_state);
        
        // Carica i landmarks dal dataset
        DataLoader loader;
        if (loader.loadLandmarkGroundtruth(dataset_path + "Landmark_Groundtruth.dat")) {
            ekf_->setLandmarks(loader.getLandmarks());
            RCLCPP_INFO(get_logger(), "Sono stati caricati %zu landmark", loader.getLandmarks().size());
        } else {
            RCLCPP_WARN(get_logger(), "Caricamento dei landmark fallito");
        }
        
        // Subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CentralizedEkfNode::odomCallback, this, _1));
        landmark_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/landmark", 10, std::bind(&CentralizedEkfNode::landmarkCallback, this, _1));
        robot_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/robot", 10, std::bind(&CentralizedEkfNode::robotCallback, this, _1));
        status_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/dataset_status", 10, std::bind(&CentralizedEkfNode::statusCallback, this, _1));
        
	// Timer per stampare a schermo periodicamente
        print_timer_ = create_wall_timer(
            std::chrono::duration<double>(print_interval_),
            std::bind(&CentralizedEkfNode::printState, this));
        RCLCPP_INFO(get_logger(), "Nodo CentralizedEKFNode initializzato per %d robot", num_robots);
        RCLCPP_INFO(get_logger(), "In attesa di dati sui seguenti topic: /odom, /landmark, /robot");
    }

private:
    /**
     * Questo callback viene chiamato ogni volta che arriva un nuovo messaggio
     * di odometria. Estrae i dati di velocità lineare e angolare,
     * calcola il tempo trascorso e applica la predizione dell'EKF.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double time = rclcpp::Time(msg->header.stamp).seconds();
        
        OdometryData odom;
        odom.timestamp = time;
        odom.v = msg->twist.twist.linear.x;
        odom.w = msg->twist.twist.angular.z;
        
        try {
            odom.robot_id = std::stoi(msg->child_frame_id);
        } catch(...) {
            RCLCPP_WARN(get_logger(), "ID del robot non valido nel messaggio di odometria, utilizzo ID 1");
            odom.robot_id = 1;
        }
        
        if(!has_time_) {
            last_time_ = time;
            has_time_ = true;
            RCLCPP_INFO(get_logger(), "Prima odometria ricevuta dal robot %d", odom.robot_id);
            return;
        }
        
        double dt = time - last_time_;
        if (dt > 0.0 && dt < 1.0) {
            ekf_->predict(odom, dt);
            
            RCLCPP_DEBUG(get_logger(), "Predizione: Robot %d, v=%.3f, w=%.3f, dt=%.3f", 
                        odom.robot_id, odom.v, odom.w, dt);
            
            measurements_since_print_++;
        }
        
        last_time_ = time;
    }
    
    /**
     * Questo callback gestisce le misurazioni range-bearing tra un robot
     * e un landmark conosciuto. Applica la correzione dell'EKF basata
     * sulla misurazione ricevuta.
     */
    void landmarkCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        MeasurementData meas;
        meas.timestamp = rclcpp::Time(msg->header.stamp).seconds();

        try {
            std::string frame_id = msg->header.frame_id;
            if (frame_id.substr(0, 6) == "robot_") {
                meas.observer_id = std::stoi(frame_id.substr(6));
            } else {
                throw std::invalid_argument("Formato frame_id non valido");
            }
        } catch(...) {
            RCLCPP_WARN(get_logger(), "ID del robot non valido nell'header del messaggio del landmark, utilizzo ID 1");
            meas.observer_id = 1;
        }
        
	meas.subject_id = static_cast<int>(msg->point.z);
        meas.range = sqrt(msg->point.x * msg->point.x + msg->point.y * msg->point.y);
        meas.bearing = atan2(msg->point.y, msg->point.x);
        meas.is_landmark = true;  

        ekf_->correct(meas);

        RCLCPP_DEBUG(get_logger(), "Correzione del landmark: Robot %d -> Landmark %d, range=%.3f, bearing=%.3f",
                    meas.observer_id, meas.subject_id, meas.range, meas.bearing);

        landmark_measurements_++;
        measurements_since_print_++;
    }
    
    /**
     * Questo callback gestisce le misurazioni range-bearing tra due robot.
     * Queste misurazioni sono fondamentali per la localizzazione cooperativa
     * poiché permettono di correggere le stime relative tra i robot.
     */
    void robotCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        MeasurementData meas;
        meas.timestamp = rclcpp::Time(msg->header.stamp).seconds();

        try {
            std::string frame_id = msg->header.frame_id;
            if (frame_id.substr(0, 6) == "robot_") {
                meas.observer_id = std::stoi(frame_id.substr(6));
            } else {
                throw std::invalid_argument("Formato frame_id non valido");
            }
        } catch(...) {
            RCLCPP_WARN(get_logger(), "ID del robot non valido nell'header del messaggio del robot, utilizzo ID 1");
            meas.observer_id = 1;
        }
        
	meas.subject_id = static_cast<int>(msg->point.z);
        meas.range = sqrt(msg->point.x * msg->point.x + msg->point.y * msg->point.y);
        meas.bearing = atan2(msg->point.y, msg->point.x);
        meas.is_landmark = false;  

        ekf_->correct(meas);

        RCLCPP_DEBUG(get_logger(), "Correzione robot-robot: Robot %d -> Robot %d, range=%.3f, bearing=%.3f",
                    meas.observer_id, meas.subject_id, meas.range, meas.bearing);

        robot_measurements_++;
        measurements_since_print_++;
    }
    
    /**
     * Questa funzione viene chiamata periodicamente dal timer.
     * Stampa le posizioni stimate di tutti i robot con le relative
     * incertezze (deviazioni standard) e statistiche sulle misurazioni.
     */
    void printState() {
        if (measurements_since_print_ == 0) {
            RCLCPP_INFO(get_logger(), "Non sono state ricevute nuove misurazione negli ultimi %.1f secondi", print_interval_);
            return;
        }

        RCLCPP_INFO(get_logger(), "\nAGGIORNAMENTO DELLO STATO EKF");
        RCLCPP_INFO(get_logger(), "Misurazioni processate dall'ultima stampa a schermo: %d", measurements_since_print_);
        RCLCPP_INFO(get_logger(), "Totale misurazioni landmark: %d", landmark_measurements_);
        RCLCPP_INFO(get_logger(), "Totale misurazioni robot-robot: %d", robot_measurements_);

        const auto& state = ekf_->getState();
        const auto& P = ekf_->getCovariance();

        int num_robots = state.size() / 3;
        for (int i = 0; i < num_robots; ++i) {
            int idx = i * 3;
            RCLCPP_INFO(get_logger(), "Robot %d: x=%.3f, y=%.3f, θ=%.3f (σx=%.3f, σy=%.3f, σθ=%.3f)",
                       i + 1,
                       state[idx], state[idx+1], state[idx+2],
                       sqrt(P(idx,idx)), sqrt(P(idx+1,idx+1)), sqrt(P(idx+2,idx+2)));
        }

        // Registra snapshot delle covarianze per analisi successiva
        ekf_->recordCovarianceSnapshot(this->now().seconds());

        measurements_since_print_ = 0;
    }
    
    /**
     * Questo callback tiene traccia di quali robot hanno terminato
     * di riprodurre i loro dataset. Quando tutti i robot hanno finito,
     * stampa lo stato finale e termina il nodo.
     */
    void statusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            finished_robots_.insert(1);
            RCLCPP_INFO(get_logger(), "Il robot ha terminato la riproduzione del suo dataset.");

            if (finished_robots_.size() >= 1) {
                RCLCPP_INFO(get_logger(), "Processing del dataset terminato! Stato EKF finale:");
                printFinalState();

                rclcpp::shutdown();
            }
        }
    }
    
    /**
     * Questa funzione viene chiamata quando tutti i robot hanno completato
     * la riproduzione dei loro dataset. Stampa le posizioni finali stimate
     * e le statistiche complete delle misurazioni processate.
     */
    void printFinalState() {
        const auto& state = ekf_->getState();
        const auto& P = ekf_->getCovariance();

        RCLCPP_INFO(get_logger(), "\nSTATO EKF FINALE");
	// Misurazioni totali 
        RCLCPP_INFO(get_logger(), "Totale misurazioni landmark processate: %d", landmark_measurements_);
        RCLCPP_INFO(get_logger(), "Totale misurazioni robot-robot processate: %d", robot_measurements_);

        int num_robots = state.size() / 3;
        for (int i = 0; i < num_robots; ++i) {
            int idx = i * 3;
            RCLCPP_INFO(get_logger(), "Robot %d FINALE: x=%.3f, y=%.3f, θ=%.3f (σx=%.3f, σy=%.3f, σθ=%.3f)",
                       i + 1,
                       state[idx], state[idx+1], state[idx+2],
                       sqrt(P(idx,idx)), sqrt(P(idx+1,idx+1)), sqrt(P(idx+2,idx+2)));
        }

        // Salva lo storico delle covarianze in un file CSV
        std::string filename = "covariance_history.csv";
        ekf_->saveCovarianceHistory(filename);
        RCLCPP_INFO(get_logger(), "\nStorico delle covarianze salvato in: %s", filename.c_str());
    }
    
    // Componenti principali
    std::unique_ptr<EkfCentralized> ekf_;    // Istanza dell'EKF centralizzato

    // Gestione del tempo
    double last_time_;                       // Timestamp dell'ultima odometria ricevuta
    bool has_time_;                          // Flag per indicare se abbiamo ricevuto il primo timestamp
    double print_interval_;                  // Intervallo di stampa dello stato (secondi)

    // Parametri del sistema 
    int num_robots_;                         // Numero di robot nel sistema
    std::set<int> finished_robots_;          // Set dei robot che hanno completato il dataset

    // Contatori per statistiche
    int measurements_since_print_ = 0;       // Numero di misurazioni dall'ultima stampa
    int landmark_measurements_ = 0;          // Totale misurazioni landmark processate
    int robot_measurements_ = 0;             // Totale misurazioni robot-robot processate

    // Subscriber ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                           // Subscriber per odometria
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr landmark_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr robot_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr print_timer_;                                                    // Timer per stampa periodica dello stato
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                              // Inizializza ROS2

    auto node = std::make_shared<CentralizedEkfNode>();    // Crea il nodo
    rclcpp::spin(node);                                    // Mantiene il nodo attivo

    rclcpp::shutdown();                                    // Chiude ROS2
    return 0;
}

