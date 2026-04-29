/**
 * Questo header definisce la classe EkfCentralized che implementa un algoritmo
 * di localizzazione cooperativa centralizzato basato su Extended Kalman Filter.
 *
 * L'approccio centralizzato mantiene una stima congiunta delle pose di tutti i
 * robot in un singolo vettore di stato, permettendo di sfruttare al meglio
 * le correlazioni tra le stime dei robot per migliorare l'accuratezza.
 *
 * Caratteristiche principali:
 * - Stato congiunto di tutti i robot: [x1,y1,θ1, x2,y2,θ2, ...]
 * - Predizione basata su modello cinematico unicycle
 * - Correzione con misurazioni landmark (assolute) e robot-robot (relative)
 * - Validazione delle misurazioni tramite distanza di Mahalanobis
 * - Gestione delle incertezze e del rumore sensore
 */

#ifndef EKF_CENTRALIZED_H
#define EKF_CENTRALIZED_H

#include <Eigen/Dense>        
#include <map>                
#include <vector>             
#include "data_loader.h"      
#include <rclcpp/rclcpp.hpp>  


/**
 * Implementa un EKF centralizzato per la localizzazione cooperativa di robot multipli.
 * Il filtro mantiene un vettore di stato congiunto che include le pose di tutti i robot
 * e utilizza sia misurazioni propriocettive (odometria) che esterocettive (range-bearing)
 * per mantenere una stima accurata delle posizioni.
 *
 * Algoritmo EKF:
 * 1. Predizione: Utilizza odometria e modello cinematico per predire il nuovo stato
 * 2. Correzione: Utilizza misurazioni per correggere la predizione
 *
 * Tipi di misurazioni supportate:
 * - Landmark: Correzioni assolute basate su punti di riferimento fissi
 * - Robot-Robot: Correzioni relative cooperative tra robot
 */
class EkfCentralized {
public:
    
    EkfCentralized(int num_robots, const Eigen::VectorXd& initial_state);


    void setLandmarks(const std::map<int, Landmark>& landmarks);


    void predict(const OdometryData& odom, double dt);

    /**
     * Predizione basata su incremento di posa.
     * Utile quando il driver ROS fornisce già lo spostamento stimato,
     * evitando problemi di sincronizzazione temporale.
     */
    void predict(const PoseIncrement& pose_inc);


    void correct(const MeasurementData& meas);

    /**
     * Correzione con vettore di misurazioni multiple.
     * Permette di processare più misurazioni in un unico ciclo di correzione,
     * utile per gestire contesti più generali rispetto al dataset UTIAS.
     */
    void correct(const std::vector<MeasurementData>& measurements);

    const Eigen::VectorXd& getState() const { return state_; }

    const Eigen::MatrixXd& getCovariance() const { return P_; }

    void printState(rclcpp::Logger logger, double timestamp) const;

    void printUncertainty(rclcpp::Logger logger) const;

    void setState(const Eigen::VectorXd& new_state);

    /**
     * Salva l'andamento delle covarianze nel tempo per monitorare la stabilità del filtro.
     * Utile per analisi della convergenza e debugging del filtro.
     */
    void saveCovarianceHistory(const std::string& filename) const;

    /**
     * Aggiunge snapshot corrente delle covarianze allo storico.
     * Da chiamare periodicamente durante l'esecuzione del filtro.
     */
    void recordCovarianceSnapshot(double timestamp);

private:
    
    double normalize_angle(double angle);

   
    bool validateMeasurement(const MeasurementData& meas, double mahalanobis_threshold = 3.0);

    // Variabili di stato dell'EKF 
    Eigen::VectorXd state_;              // Vettore di stato [x1,y1,θ1, x2,y2,θ2, ...]
    Eigen::MatrixXd P_;                  // Matrice di covarianza dello stato
    Eigen::MatrixXd Q_;                  // Matrice di rumore di processo
    Eigen::MatrixXd R_landmark_;         // Matrice di rumore per misurazioni landmark
    Eigen::MatrixXd R_robot_;            // Matrice di rumore per misurazioni robot-robot

    // Parametri del sistema
    int num_robots_;                     	// Numero di robot nel sistema
    std::map<int, Landmark> landmark_map_;	// Mappa dei landmark conosciuti
    double last_correction_time_ = -1;   	// Timestamp dell'ultima correzione

    // Storico covarianze
    struct CovarianceSnapshot {
        double timestamp;
        std::vector<double> sigma_x;     // Deviazioni standard x per ogni robot
        std::vector<double> sigma_y;     // Deviazioni standard y per ogni robot
        std::vector<double> sigma_theta; // Deviazioni standard theta per ogni robot
    };
    std::vector<CovarianceSnapshot> covariance_history_; // Storico covarianze nel tempo
};

#endif // EKF_CENTRALIZED_H
