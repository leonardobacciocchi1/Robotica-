/**
 * Questo header definisce:
 * - Strutture dati per odometria, misurazioni, ground truth e landmark
 * - Classe DataLoader per il caricamento dati dal dataset MRCLAM
 * - Tipi e utilities per la gestione degli eventi temporali
 *
 * Il dataset MRCLAM è un dataset standard per SLAM cooperativo che contiene
 * dati di 5 robot che si muovono in un ambiente con landmark.
 */

#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <string>       
#include <vector>       
#include <variant>      
#include <map>          
#include <iostream>     
#include <fstream>      
#include <sstream>      
#include <algorithm>    
#include <Eigen/Dense>  

/**
 * Contiene le informazioni di velocità misurate dai sensori propriocettivi
 * del robot (encoder delle ruote, IMU, ecc.). Utilizzata per la predizione
 * nell'EKF attraverso il modello cinematico dell'unicycle.
 */
struct OdometryData {
    double timestamp;  // Timestamp della misurazione (s)
    double v;          // Velocità lineare del robot (m/s)
    double w;          // Velocità angolare del robot (rad/s)
    int robot_id;      // ID del robot (1-5)
};


/**
 * Contiene lo spostamento incrementale della posa del robot tra due istanti.
 * Questa rappresentazione è utile quando si lavora con robot reali dove il
 * driver ROS fornisce già la stima dello spostamento, evitando problemi di
 * sincronizzazione tra il timer del computer e quello della scheda di controllo.
 */
struct PoseIncrement {
    double timestamp;  // Timestamp della misurazione (s)
    double dx;         // Incremento in x nel sistema di riferimento del robot (m)
    double dy;         // Incremento in y nel sistema di riferimento del robot (m)
    double dtheta;     // Incremento angolare (rad)
    int robot_id;      // ID del robot (1-5)
};


/**
 * Rappresenta una misurazione di distanza e angolo effettuata da un robot
 * verso un landmark o un altro robot. Queste misurazioni sono utilizzate
 * per la correzione nell'EKF e sono fondamentali per la localizzazione
 * cooperativa.
 */
struct MeasurementData {
    double timestamp;   // Timestamp della misurazione (s)
    int observer_id;    // ID del robot che effettua la misurazione
    int subject_id;     // ID dell'oggetto osservato (landmark o robot)
    double range;       // Distanza misurata (m)
    double bearing;     // Angolo misurato rispetto all'orientamento del robot (rad)
    bool is_landmark;   // True se il subject è un landmark, false se è un robot
};


/**
 * Contiene la posizione e orientamento reali di un robot in un dato istante.
 * Utilizzata per valutare le performance dell'algoritmo di localizzazione
 * confrontando le stime con i valori reali.
 */
struct Groundtruth {
    double timestamp;        // Timestamp del punto traiettoria (s)
    Eigen::Vector3d pose;    // Posa reale [x, y, theta] (m, m, rad)
};


/**
 * Rappresenta un punto di riferimento fisso nell'ambiente la cui posizione
 * è nota con precisione. I landmark forniscono correzioni assolute all'EKF
 * permettendo di ridurre l'accumulo di errori della localizzazione.
 */
struct Landmark {
    int id;                  // ID univoco del landmark
    Eigen::Vector2d pos;     // Posizione cartesiana [x, y] (m)
};


/**
 * Utilizza std::variant per creare un tipo che può contenere sia dati
 * di odometria che di misurazione. Questo permette di gestire tutti gli
 * eventi in un'unica coda temporale ordinata, essenziale per la corretta
 * riproduzione sincronizzata dei dati del dataset.
 */
using Event = std::variant<OdometryData, MeasurementData>;


/**
 * Fornisce un operatore di confronto che permette di ordinare eventi
 * di tipo diverso (odometria e misurazioni) basandosi sul loro timestamp.
 * Utilizza std::visit per estrarre il timestamp da oggetti std::variant.
 */
struct EventComparator {    
    bool operator()(const Event& a, const Event& b) const;
};


/**
 * Questa classe si occupa di:
 * - Caricare tutti i tipi di dati dal dataset MRCLAM
 * - Convertire i barcode in ID reali degli oggetti
 * - Mantenere una coda ordinata di eventi temporali
 * - Fornire accesso ai landmark dell'ambiente
 *
 * Il dataset MRCLAM organizza i dati in file separati per ogni robot
 * e tipo di dato (odometria, misurazioni, groundtruth).
 */
class DataLoader {
public:
    
    bool loadLandmarkGroundtruth(const std::string& filename);
    bool loadBarcodes(const std::string& filename);
    bool loadOdometry(const std::string& filename, int robot_id);
    bool loadMeasurements(const std::string& filename, int robot_id);
    const std::vector<Event>& getEventQueue() const;
    const std::map<int, Landmark>& getLandmarks() const;

private:
    std::vector<Event> event_queue;            // Coda ordinata di tutti gli eventi
    std::map<int, Landmark> landmark_map;      // Mappa ID -> posizione landmark
    std::map<int, int> barcode_to_subject_;    // Mappatura barcode -> ID reale
};

Eigen::VectorXd getInitialStateFromGroundtruth(const std::string& dataset_path, int num_robots);

#endif // DATA_LOADER_H
