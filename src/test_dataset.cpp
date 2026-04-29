/**
 * @file test_dataset.cpp
 * @brief Implementazione del test per la simulazione e valutazione di un filtro EKF centralizzato
 * 
 * Questo file carica i dati del dataset MRCLAM (UTIAS), tra cui barcode,
 * landmark, odometria e misurazioni per più robot, e li utilizza per
 * eseguire una simulazione con un filtro di Kalman esteso (EKF) centralizzato.
 * 
 * Durante l'esecuzione :
 * - Carica e valida i dati del dataset per due robot (1 e 2)
 * - Inizializza il filtro EKF centralizzato
 * - Esegue i passi di predizione e correzione in base agli eventi del dataset
 * - Mostra a schermo i risultati intermedi e finali di stato e incertezza
 * 
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */
 
#include "data_loader.h"
#include "ekf_centralized.h"
#include <iostream>
#include <chrono>

int main() {
    DataLoader loader;
    
    // Carica i dati del dataset UTIAS
    std::string dataset_path = "MRCLAM1/MRCLAM_Dataset1/";
    
    std::cout << "Caricamento dati dataset UTIAS..." << std::endl;
    
    // Carica i barcode e i landmark
    if (!loader.loadBarcodes(dataset_path + "Barcodes.dat")) {
        std::cerr << "Errore nel caricamento dei barcode" << std::endl;
        return -1;
    }
    
    if (!loader.loadLandmarkGroundtruth(dataset_path + "Landmark_Groundtruth.dat")) {
        std::cerr << "Errore nel caricamento dei landmark" << std::endl;
        return -1;
    }
    
    // Carica odometria per robot 1 e 2 (esempio con 2 robot)
    if (!loader.loadOdometry(dataset_path + "Robot1_Odometry.dat", 1)) {
        std::cerr << "Errore nel caricamento odometria Robot 1" << std::endl;
        return -1;
    }
    
    if (!loader.loadOdometry(dataset_path + "Robot2_Odometry.dat", 2)) {
        std::cerr << "Errore nel caricamento odometria Robot 2" << std::endl;
        return -1;
    }
    
    // Carica misure per robot 1 e 2
    if (!loader.loadMeasurements(dataset_path + "Robot1_Measurement.dat", 1)) {
        std::cerr << "Errore nel caricamento misure Robot 1" << std::endl;
        return -1;
    }
    
    if (!loader.loadMeasurements(dataset_path + "Robot2_Measurement.dat", 2)) {
        std::cerr << "Errore nel caricamento misure Robot 2" << std::endl;
        return -1;
    }
    
    // Inizializza EKF centralizzato per 2 robot
    Eigen::VectorXd initial_state(6);
    initial_state << 0.0, 0.0, 0.0,  // Robot 1: x, y, theta
                     1.0, 1.0, 0.0;  // Robot 2: x, y, theta
    
    EkfCentralized ekf(2, initial_state);
    ekf.setLandmarks(loader.getLandmarks());
    
    std::cout << "EKF centralizzato inizializzato con " << loader.getLandmarks().size() 
              << " landmark" << std::endl;
    
    // Processa gli eventi in ordine temporale
    const auto& events = loader.getEventQueue();
    std::cout << "Eventi totali da processare: " << events.size() << std::endl;
    
    double last_timestamp = 0.0;
    int odom_count = 0, meas_count = 0;
    
    for (size_t i = 0; i < std::min(events.size(), size_t(1000)); ++i) {
        const Event& event = events[i];
        
        std::visit([&](const auto& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, OdometryData>) {
                double dt = (last_timestamp > 0) ? data.timestamp - last_timestamp : 0.1;
                ekf.predict(data, dt);
                odom_count++;
                
                if (odom_count % 10 == 0) {
                    std::cout << "Odometria Robot " << data.robot_id 
                              << " @ t=" << std::fixed << std::setprecision(3) << data.timestamp
                              << " v=" << data.v << " w=" << data.w << std::endl;
                }
            } else if constexpr (std::is_same_v<T, MeasurementData>) {
                ekf.correct(data);
                meas_count++;
                
                if (meas_count % 5 == 0) {
                    std::cout << "Misura Robot " << data.observer_id 
                              << " -> Soggetto " << data.subject_id
                              << " @ t=" << std::fixed << std::setprecision(3) << data.timestamp
                              << " range=" << data.range << " bearing=" << data.bearing << std::endl;
                }
            }
            last_timestamp = data.timestamp;
        }, event);
    }
    
    std::cout << "\nRISULTATI FINALI" << std::endl;
    std::cout << "Eventi processati: " << std::min(events.size(), size_t(1000)) << std::endl;
    std::cout << "- Odometria: " << odom_count << std::endl;
    std::cout << "- Misure: " << meas_count << std::endl;
    
    std::cout << "\nStato finale dell'EKF:" << std::endl;
    const auto& final_state = ekf.getState();
    std::cout << "Robot 1: x=" << final_state[0] << " y=" << final_state[1] << " θ=" << final_state[2] << std::endl;
    std::cout << "Robot 2: x=" << final_state[3] << " y=" << final_state[4] << " θ=" << final_state[5] << std::endl;
    
    std::cout << "\nIncertezza finale:" << std::endl;
    const auto& P = ekf.getCovariance();
    std::cout << "Robot 1 - σx=" << sqrt(P(0,0)) << " σy=" << sqrt(P(1,1)) << " σθ=" << sqrt(P(2,2)) << std::endl;
    std::cout << "Robot 2 - σx=" << sqrt(P(3,3)) << " σy=" << sqrt(P(4,4)) << " σθ=" << sqrt(P(5,5)) << std::endl;
    
    return 0;
}
