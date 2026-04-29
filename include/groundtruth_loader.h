/**
 * Questo header definisce funzioni di utilità per caricare e utilizzare
 * i dati groundtruth dal dataset MRCLAM.
 *
 * I dati groundtruth rappresentano le posizioni e traiettorie reali
 * dei robot, ottenute attraverso un sistema di motion capture o GPS
 * ad alta precisione. Sono utilizzati per:
 *
 * - Inizializzazione: Fornire uno stato iniziale preciso all'EKF
 * - Valutazione: Calcolare metriche di errore delle stime
 * - Debugging: Confrontare le stime con i valori reali
 */
 
#ifndef GROUNDTRUTH_LOADER_H
#define GROUNDTRUTH_LOADER_H

#include <Eigen/Dense>  
#include <string>       

/**
 * Questa funzione:
 * 1. Cerca i file RobotX_Groundtruth.dat per ogni robot
 * 2. Estrae la prima posa di ogni file come stato iniziale
 * 3. Costruisce il vettore di stato completo per l'EKF centralizzato
 * 4. In caso di file mancanti, utilizza valori di default (0,0,0)
 *
 * Il vettore risultante ha dimensione 3*num_robots e contiene le pose iniziali di tutti i robot nell'ordine:
 * [x_robot1, y_robot1, theta_robot1, x_robot2, y_robot2, theta_robot2, ...]
 */
Eigen::VectorXd getInitialStateFromGroundtruth(const std::string& dataset_path, int num_robots);

#endif // GROUNDTRUTH_LOADER_H
