/**
 * @file robot_node.cpp
 * @brief Implementazione nodo ROS2 che simula un robot del dataset
 * 
 * Questo file implementa un nodo ROS2 che simula un robot durante l'acquisizione
 * di dati sensoriali. Il nodo legge i dati da file del dataset MRCLAM e li
 * ripubblica come messaggi ROS2 con timing appropriato per simulare l'esecuzione in tempo reale.
 *
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"  
#include "data_loader.h"                                   
#include <chrono>                                          
#include <thread>                                          
using namespace std::chrono_literals;

/**
 * Questa classe eredita da rclcpp::Node e simula un robot che acquisisce e pubblica dati sensoriali. 
 * Il nodo :
 * - Carica i dati di odometria e misurazioni dal dataset
 * - Pubblica i dati con timing corretto per simulare l'esecuzione reale
 * - Gestisce landmark e misurazioni robot-robot separatamente
 * - Notifica quando ha terminato di processare tutti i dati
 */
class RobotNode : public rclcpp::Node {
public:
    /**
     * Inizializza il nodo, configura i publisher, carica i dati del dataset
     * specifico per questo robot e avvia la pubblicazione temporizzata.
     */
    RobotNode(int robot_id) : Node("robot_" + std::to_string(robot_id) + "_node"),
                              robot_id_(robot_id) {
        
        // Publisher
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        landmark_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/landmark", 10);
        robot_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/robot", 10);
        status_pub_ = create_publisher<std_msgs::msg::Bool>("/dataset_status", 10);
        
        // Parametri
        declare_parameter("dataset_path", std::string("MRCLAM1/MRCLAM_Dataset1/"));
        declare_parameter("playback_speed", 1.0);
        
        std::string dataset_path;
        get_parameter("dataset_path", dataset_path);
        get_parameter("playback_speed", playback_speed_);
        
        // Caricamento del dataset
        if (!loadRobotData(dataset_path)) {
            RCLCPP_ERROR(get_logger(), "Caricamento del dataset fallito per il robot %d", robot_id_);
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Il robot %d ha caricato %zu eventi", robot_id_, events_.size());
        
        // Avvio pubblicazione
        timer_ = create_wall_timer(10ms, std::bind(&RobotNode::publishData, this));
        
        current_event_ = 0;
        start_time_ = now();
        if (!events_.empty()) {
            auto first_timestamp = std::visit([](const auto& event) { return event.timestamp; }, events_[0]);
            dataset_start_time_ = first_timestamp;
        }
    }

private:
    /**
     * Carica la mappatura dei barcode, i dati di odometria e le misurazioni
     * per questo specifico robot dal dataset MRCLAM.
     */
    bool loadRobotData(const std::string& dataset_path) {
        DataLoader loader;
        
        // Caricamento mappatura barcode
        if (!loader.loadBarcodes(dataset_path + "Barcodes.dat")) {
            return false;
        }
        
        // Caricamento dati specifici del robot
        std::string odom_file = dataset_path + "Robot" + std::to_string(robot_id_) + "_Odometry.dat";
        std::string meas_file = dataset_path + "Robot" + std::to_string(robot_id_) + "_Measurement.dat";
        
        if (!loader.loadOdometry(odom_file, robot_id_)) {
            RCLCPP_ERROR(get_logger(), "Caricamento odometria fallito da %s", odom_file.c_str());
            return false;
        }
        
        if (!loader.loadMeasurements(meas_file, robot_id_)) {
            RCLCPP_WARN(get_logger(), "Caricamento odometria fallito da %s", meas_file.c_str());
            
            // Continua comunque, infatti alcuni robot potrebbero non avere misurazioni
        }
        
        events_ = loader.getEventQueue();
        return true;
    }
    
    /**
     * Questa funzione viene chiamata periodicamente dal timer.
     * Confronta il tempo reale trascorso con i timestamp del dataset
     * e pubblica tutti gli eventi che dovrebbero essere accaduti.
     * Gestisce la velocità di riproduzione configurabile.
     */
    void publishData() {
        if (current_event_ >= events_.size()) {
            return; // Tutti gli eventi pubblicati
        }
        
        auto current_ros_time = now();
        double elapsed_real_time = (current_ros_time - start_time_).seconds() * playback_speed_;
        
        // Pubblica tutti gli eventi che dovrebbero essere accaduti fino ad ora
        while (current_event_ < events_.size()) {
            const auto& event = events_[current_event_];
            
            double event_timestamp = std::visit([](const auto& e) { return e.timestamp; }, event);
            double event_offset = event_timestamp - dataset_start_time_;
            
            if (event_offset > elapsed_real_time) {
                break; // L’evento è nel futuro
            }
            
            // Pubblica l'evento
            std::visit([this, &current_ros_time](const auto& data) {
                using T = std::decay_t<decltype(data)>;
                
                if constexpr (std::is_same_v<T, OdometryData>) {
                    publishOdometry(data, current_ros_time);
                } else if constexpr (std::is_same_v<T, MeasurementData>) {
                    publishMeasurement(data, current_ros_time);
                }
            }, event);
            
            current_event_++;
        }
        
        if (current_event_ >= events_.size()) {
            RCLCPP_INFO(get_logger(), "Robot %d ha terminato la pubblicazione di tutti i %zu eventi", 
                       robot_id_, events_.size());
            
            // Pubblica messaggio di completamento
            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = true;
            status_pub_->publish(status_msg);
            
            timer_->cancel();
            
            // Ritardo per garantire l’invio del messaggio, poi spegnimento
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::shutdown();
        }
    }
    
    /**
     * Converte i dati di odometria dal formato interno al messaggio
     * ROS2 standard nav_msgs::msg::Odometry e lo pubblica.
     */
    void publishOdometry(const OdometryData& odom, rclcpp::Time stamp) {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = stamp;
        msg.header.frame_id = "odom";
        msg.child_frame_id = std::to_string(robot_id_);
        
        msg.twist.twist.linear.x = odom.v;
        msg.twist.twist.angular.z = odom.w;
        
        odom_pub_->publish(msg);
        
        RCLCPP_DEBUG(get_logger(), "Robot %d ha pubblicato odometria: v=%.3f, w=%.3f", 
                     robot_id_, odom.v, odom.w);
    }
    
    /**
     * Converte i dati della misurazione dal formato interno al messaggio
     * personalizzato e determina se pubblicarla sul topic landmark o robot
     */
    void publishMeasurement(const MeasurementData& meas, rclcpp::Time stamp) {
        auto msg = geometry_msgs::msg::PointStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = "robot_" + std::to_string(robot_id_);
        msg.point.x = meas.range * cos(meas.bearing);
        msg.point.y = meas.range * sin(meas.bearing);
        msg.point.z = static_cast<double>(meas.subject_id);

        if (meas.is_landmark) {
            landmark_pub_->publish(msg);
            RCLCPP_DEBUG(get_logger(), "Robot %d ha pubblicato misurazione landmark: landmark %d range=%.3f, bearing=%.3f",
                         robot_id_, meas.subject_id, meas.range, meas.bearing);
        } else {
            robot_pub_->publish(msg);
            RCLCPP_DEBUG(get_logger(), "Robot %d ha pubblicato misurazione robot: robot %d range=%.3f, bearing=%.3f",
                         robot_id_, meas.subject_id, meas.range, meas.bearing);
        }
    }
    
    // Identificazione robot
    int robot_id_;                         

    // Gestione eventi dataset
    std::vector<Event> events_;              // Coda ordinata di tutti gli eventi (odometria + misurazioni)
    size_t current_event_;                   // Indice dell'evento corrente da pubblicare
    double playback_speed_;                  // Velocità di riproduzione (1.0 = tempo reale)

    // Gestione temporale
    double dataset_start_time_;              // Timestamp del primo evento nel dataset
    rclcpp::Time start_time_;                // Tempo di inizio della riproduzione

    // Publisher ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;                           // Publisher odometria
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr landmark_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr robot_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;     // Timer per pubblicazione periodica dei dati
};

//Parsa l'ID del robot dai parametri della riga di comando, valida l'input, crea un'istanza del nodo RobotNode e lo mantiene attivo.
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                              // Inizializza ROS2

    // Verifica che sia stato fornito l'ID del robot
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <robot_id>" << std::endl;
        return -1;
    }

    // Parsa e valida l'ID del robot
    int robot_id = std::atoi(argv[1]);
    if (robot_id < 1 || robot_id > 5) {
        std::cerr << "L'ID del robot deve essere compreso tra 1 e 5." << std::endl;
        return -1;
    }

    auto node = std::make_shared<RobotNode>(robot_id);     // Crea il nodo specifico per il robot
    rclcpp::spin(node);                                    // Mantiene il nodo attivo
    rclcpp::shutdown();                                    // Chiude ROS2

    return 0;
}
