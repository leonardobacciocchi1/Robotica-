/**
 * @file evaluate_accuracy.cpp
 * @brief Valutazione dell’accuratezza del filtro EKF centralizzato su dataset MRCLAM.
 *
 * Questo file implementa il programma principale per la valutazione delle prestazioni
 * di un filtro di Kalman esteso (EKF) centralizzato in un contesto di localizzazione
 * cooperativa multi-robot.
 *
 * Il codice :
 * - Carica i dati di ground truth dei robot
 * - Carica e organizza i dati del dataset MRCLAM (odometria, landmark, misurazioni)
 * - Inizializza l’EKF con lo stato iniziale derivato dal ground truth
 * - Processa tutti gli eventi temporali (predizione e correzione)
 * - Confronta le stime finali dell’EKF con i dati di ground truth
 * - Calcola metriche di accuratezza (errori di posizione e orientamento)
 * - Stampa un resoconto delle prestazioni complessive
 *
 * L’obiettivo di questo file è analizzare quantitativamente l’accuratezza
 * del filtro EKF centralizzato applicato a dati reali provenienti dal dataset MRCLAM,
 * valutandone la precisione e la robustezza.
 *
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */


#include "data_loader.h"
#include "ekf_centralized.h"
#include "groundtruth_loader.h"
#include <iostream>
#include <iomanip>
#include <cmath>

struct RobotGroundtruth {
    double timestamp;
    double x, y, theta;
};

/**
 * Questa funzione legge un file contenente i dati di ground truth (timestamp, x, y, theta)
 * di un robot e li memorizza in un vettore di strutture RobotGroundtruth.
 */
std::vector<RobotGroundtruth> loadGroundtruth(const std::string& filename) {
    std::vector<RobotGroundtruth> gt_data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Impossibile aprire il file dei groundtruth: " << filename << std::endl;
        return gt_data;
    }
    
    std::string line;
    // Ignora i commenti
    while (std::getline(file, line) && line.find('#') != std::string::npos);
    
    do {
        if (line.empty()) continue;
        std::stringstream ss(line);
        RobotGroundtruth gt;
        ss >> gt.timestamp >> gt.x >> gt.y >> gt.theta;
        gt_data.push_back(gt);
    } while (std::getline(file, line));
    
    return gt_data;
}

/**
 * Questa funzione assicura che l’angolo fornito sia compreso nel range canonico
 * [-π, π], utile per confrontare o calcolare differenze angolari.
 */
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * Funzione principale per la valutazione delle prestazioni del EKF :
 * - Carica i dati di ground truth e del dataset MRCLAM.
 * - Inizializza un filtro EKF centralizzato con lo stato iniziale.
 * - Processa in sequenza gli eventi temporali (odometria e misurazioni).
 * - Confronta lo stato finale stimato con il ground truth.
 * - Calcola e stampa le metriche di accuratezza (errori e incertezze).
 */
int main() {
    DataLoader loader;
    std::string dataset_path = "MRCLAM1/MRCLAM_Dataset1/";
    
    // Carica i dati di ground truth
    auto robot1_gt = loadGroundtruth(dataset_path + "Robot1_Groundtruth.dat");
    auto robot2_gt = loadGroundtruth(dataset_path + "Robot2_Groundtruth.dat");
    
    if (robot1_gt.empty() || robot2_gt.empty()) {
        std::cerr << "Errore durante il caricamento dei dati di groundtruth" << std::endl;
        return -1;
    }
    
    std::cout << "Caricati " << robot1_gt.size() << " punti di groundtruth per il Robot 1" << std::endl;
    std::cout << "Caricati " << robot2_gt.size() << " punti di groundtruth per il Robot 2" << std::endl;
    
    // Carica il dataset per la simulazione EKF
    if (!loader.loadBarcodes(dataset_path + "Barcodes.dat") ||
        !loader.loadLandmarkGroundtruth(dataset_path + "Landmark_Groundtruth.dat") ||
        !loader.loadOdometry(dataset_path + "Robot1_Odometry.dat", 1) ||
        !loader.loadOdometry(dataset_path + "Robot2_Odometry.dat", 2) ||
        !loader.loadMeasurements(dataset_path + "Robot1_Measurement.dat", 1) ||
        !loader.loadMeasurements(dataset_path + "Robot2_Measurement.dat", 2)) {
        std::cerr << "Errore durante il caricamento del dataset" << std::endl;
        return -1;
    }
    
    // Inizializza l’EKF con le posizioni iniziali dal ground truth
    Eigen::VectorXd initial_state = getInitialStateFromGroundtruth(dataset_path, 2);
    
    EkfCentralized ekf(2, initial_state);
    ekf.setLandmarks(loader.getLandmarks());
    
    std::cout << "\nStato iniziale dal groundtruth:" << std::endl;
    std::cout << "Robot 1: x=" << robot1_gt[0].x << " y=" << robot1_gt[0].y << " θ=" << robot1_gt[0].theta << std::endl;
    std::cout << "Robot 2: x=" << robot2_gt[0].x << " y=" << robot2_gt[0].y << " θ=" << robot2_gt[0].theta << std::endl;
    
    // Processa gli eventi
    const auto& events = loader.getEventQueue();
    double last_timestamp = 0.0;
    size_t events_processed = 0;
    
    // Processa tutti gli eventi (può essere limitato se troppi)
    size_t max_events = std::min(events.size(), size_t(50000));
    
    for (size_t i = 0; i < max_events; ++i) {
        const Event& event = events[i];
        
        std::visit([&](const auto& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, OdometryData>) {
                double dt = (last_timestamp > 0) ? data.timestamp - last_timestamp : 0.1;
                ekf.predict(data, dt);
            } else if constexpr (std::is_same_v<T, MeasurementData>) {
                ekf.correct(data);
            }
            last_timestamp = data.timestamp;
        }, event);
        
        events_processed++;
        if (events_processed % 10000 == 0) {
            std::cout << "Processati " << events_processed << " eventi..." << std::endl;
        }
    }
    
    // Ottieni lo stato finale del EKF
    const auto& final_state = ekf.getState();
    const auto& final_cov = ekf.getCovariance();
    
    // Trova i timestamp di ground truth più vicini
    auto& robot1_final_gt = robot1_gt.back();
    auto& robot2_final_gt = robot2_gt.back();
    
    std::cout << "\nVALUTAZIONE ACCURATEZZA FINALE" << std::endl;
    std::cout << "Eventi processati: " << events_processed << " / " << events.size() << std::endl;
    std::cout << "Timestamp finale: " << std::fixed << std::setprecision(3) << last_timestamp << std::endl;
    
    // Calcolo errori
    double robot1_error_x = final_state[0] - robot1_final_gt.x;
    double robot1_error_y = final_state[1] - robot1_final_gt.y;
    double robot1_error_theta = normalizeAngle(final_state[2] - robot1_final_gt.theta);
    double robot1_error_pos = sqrt(robot1_error_x*robot1_error_x + robot1_error_y*robot1_error_y);
    
    double robot2_error_x = final_state[3] - robot2_final_gt.x;
    double robot2_error_y = final_state[4] - robot2_final_gt.y;
    double robot2_error_theta = normalizeAngle(final_state[5] - robot2_final_gt.theta);
    double robot2_error_pos = sqrt(robot2_error_x*robot2_error_x + robot2_error_y*robot2_error_y);
    
    std::cout << "\nACCURATEZZA ROBOT 1:" << std::endl;
    std::cout << "Ground truth: x=" << robot1_final_gt.x << " y=" << robot1_final_gt.y << " θ=" << robot1_final_gt.theta << std::endl;
    std::cout << "Stima EKF: x=" << final_state[0] << " y=" << final_state[1] << " θ=" << final_state[2] << std::endl;
    std::cout << "Errori: Δx=" << robot1_error_x << "m, Δy=" << robot1_error_y << "m, Δθ=" << robot1_error_theta << "rad" << std::endl;
    std::cout << "Errore di posizione: " << robot1_error_pos << "m" << std::endl;
    std::cout << "Errore angolare: " << fabs(robot1_error_theta) << "rad (" << fabs(robot1_error_theta) * 180/M_PI << "°)" << std::endl;
    std::cout << "Incertezza: σx=" << sqrt(final_cov(0,0)) << " σy=" << sqrt(final_cov(1,1)) << " σθ=" << sqrt(final_cov(2,2)) << std::endl;
    
    std::cout << "\nACCURATEZZA ROBOT 2:" << std::endl;
    std::cout << "Ground truth: x=" << robot2_final_gt.x << " y=" << robot2_final_gt.y << " θ=" << robot2_final_gt.theta << std::endl;
    std::cout << "Stima EKF: x=" << final_state[3] << " y=" << final_state[4] << " θ=" << final_state[5] << std::endl;
    std::cout << "Errori: Δx=" << robot2_error_x << "m, Δy=" << robot2_error_y << "m, Δθ=" << robot2_error_theta << "rad" << std::endl;
    std::cout << "Errore di posizione: " << robot2_error_pos << "m" << std::endl;
    std::cout << "Errore angolare: " << fabs(robot2_error_theta) << "rad (" << fabs(robot2_error_theta) * 180/M_PI << "°)" << std::endl;
    std::cout << "Incertezza: σx=" << sqrt(final_cov(3,3)) << " σy=" << sqrt(final_cov(4,4)) << " σθ=" << sqrt(final_cov(5,5)) << std::endl;
    
    
    // Metriche di prestazione complessive
    double avg_pos_error = (robot1_error_pos + robot2_error_pos) / 2.0;
    double avg_ang_error = (fabs(robot1_error_theta) + fabs(robot2_error_theta)) / 2.0;
    
    std::cout << "\nPRESTAZIONI GLOBALI" << std::endl;
    std::cout << "Errore medio posizione: " << avg_pos_error << "m" << std::endl;
    std::cout << "Errore medio angolare: " << avg_ang_error << "rad (" << avg_ang_error * 180/M_PI << "°)" << std::endl;
    
    // Valutazione delle prestazioni
    std::cout << "\nVALUTAZIONE DELLE PRESTAZIONI" << std::endl;
    if (avg_pos_error < 0.5) {
        std::cout << "ACCURATEZZA di posizione ECCELLENTE (< 0.5m)" << std::endl;
    } else if (avg_pos_error < 1.0) {
        std::cout << "BUONA accuratezza di posizione (< 1.0m)" << std::endl;
    } else if (avg_pos_error < 2.0) {
        std::cout << "ACCURATEZZA di posizione ACCETTABILE (< 2.0m)" << std::endl;
    } else {
        std::cout << "ACCURATEZZA di posizione SCARSA (> 2.0m)" << std::endl;
    }
    
    if (avg_ang_error * 180/M_PI < 10.0) {
        std::cout << "ACCURATEZZA angolare ECCELLENTE (< 10°)" << std::endl;
    } else if (avg_ang_error * 180/M_PI < 20.0) {
        std::cout << "BUONA accuratezza angolare (< 20°)" << std::endl;
    } else if (avg_ang_error * 180/M_PI < 45.0) {
        std::cout << "ACCURATEZZA angolare ACCETTABILE (< 45°)" << std::endl;
    } else {
        std::cout << "ACCURATEZZA angolare SCARSA (> 45°)" << std::endl;
    }
    
    return 0;
}
