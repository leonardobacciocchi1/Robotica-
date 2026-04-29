/**
 * @file groundtruth_loader.cpp
 * @brief Implementazione delle funzioni per caricare il dati di groundtruth dei robot.
 *
 * Questo file contiene funzioni di utilità per caricare le traiettorie
 * groundtruth dei robot dal dataset MRCLAM. Questi dati rappresentano
 * le posizioni reali dei robot e sono utilizzati per:
 * - Inizializzazione dello stato iniziale dell'EKF
 * - Valutazione delle performance dell'algoritmo
 * - Confronto con le stime ottenute
 *
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */

#include "data_loader.h"
#include <fstream>     
#include <sstream>   
#include <iostream>     

/**
 * Rappresenta un singolo punto temporale della traiettoria reale di un robot,
 * contenente timestamp e posa completa (posizione + orientamento).
 */
struct GroundtruthData {
    double timestamp;  // Timestamp del punto traiettoria
    double x, y;       // Posizione cartesiana (x, y)
    double theta;      // Orientamento (angolo in rad)
};

/**
 * Legge un file di ground truth nel formato standard MRCLAM :
 * - timestamp
 * - x 
 * - y
 * - theta
 * Ogni riga rappresenta una posa del robot in un istante temporale specifico.
 */
std::vector<GroundtruthData> loadGroundtruthFile(const std::string& filename) {
    std::vector<GroundtruthData> gt_data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Impossibile aprire il file dei groundtruth: " << filename << std::endl;
        return gt_data;  // Ritorna vettore vuoto in caso di errore
    }

    std::string line;
    // Salta le righe di commento
    while (std::getline(file, line) && line.find('#') != std::string::npos);

    // Legge ogni riga di dati groundtruth
    do {
        if (line.empty()) continue;  // Salta righe vuote
        std::stringstream ss(line);
        GroundtruthData gt;
        ss >> gt.timestamp >> gt.x >> gt.y >> gt.theta;  // Parsing dati
        gt_data.push_back(gt);  // Aggiunge al vettore
    } while (std::getline(file, line));

    return gt_data;
}

/**
 * Per ogni robot, carica il file ground truth corrispondente e estrae
 * la prima posa come stato iniziale. Questo fornisce un'inizializzazione
 * precisa per l'EKF invece di usare valori di default.
 */
Eigen::VectorXd getInitialStateFromGroundtruth(const std::string& dataset_path, int num_robots) {
    Eigen::VectorXd initial_state(num_robots * 3);  // 3 stati per robot

    // Cicla attraverso tutti i robot
    for (int i = 1; i <= num_robots; ++i) {
        // Costruisce il nome del file ground truth per questo robot
        std::string gt_file = dataset_path + "Robot" + std::to_string(i) + "_Groundtruth.dat";
        auto gt_data = loadGroundtruthFile(gt_file);

        if (!gt_data.empty()) {
            // Usa la prima posa del ground truth come stato iniziale
            int idx = (i - 1) * 3;  // Indice nel vettore di stato
            initial_state[idx] = gt_data[0].x;         // Posizione x iniziale
            initial_state[idx + 1] = gt_data[0].y;     // Posizione y iniziale
            initial_state[idx + 2] = gt_data[0].theta; // Orientamento θ iniziale

            std::cout << "Robot " << i << "groundtruth inziale: x=" << gt_data[0].x << " y=" << gt_data[0].y << " θ=" << gt_data[0].theta << std::endl;
        } else {
            // Se non è disponibile il groundtruth, usa valori di default
            std::cerr << "Attenzione: non è disponibili i dati di groundtruth per il robot " << i << ", vengono usati i valori di default (0,0,0)" << std::endl;
            int idx = (i - 1) * 3;
            initial_state[idx] = 0.0;     // x = 0
            initial_state[idx + 1] = 0.0; // y = 0
            initial_state[idx + 2] = 0.0; // θ = 0
        }
    }

    return initial_state;
}
