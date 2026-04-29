/**
 * @file data_loader.cpp
 * @brief Implementazione della classe DataLoader per la gestione dei dati del dataset.
 *
 * Questo file implementa la classe DataLoader che si occupa di:
 * - Caricare la mappatura barcode-landmark
 * - Caricare i dati di ground truth dei landmark
 * - Caricare dati di odometria dai file del dataset
 * - Caricare misurazioni range-bearing dai file del dataset
 * - Organizzare tutti gli eventi in una coda temporale ordinata
 *
 *
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */

#include "data_loader.h"

/**
 * Utilizza std::visit per estrarre il timestamp da varianti di eventi
 * (OdometryData o MeasurementData) e li confronta per ordinamento temporale.
 */
bool EventComparator::operator()(const Event& a, const Event& b) const {
    auto get_timestamp = [](const Event& e) {
        return std::visit([](const auto& arg) { return arg.timestamp; }, e);
    };
    return get_timestamp(a) < get_timestamp(b);
}

/**
 * Il file dei barcode contiene la mappatura tra i codici letti dai sensori
 * e gli ID reali dei landmark o robot. Ogni riga contiene : 
 *	- subject_id
 *	- barcode_id
 */
bool DataLoader::loadBarcodes(const std::string& filename) {
    std::ifstream file(filename);
    if(!file.is_open()) {
        std::cerr << "Errore: Impossibile aprire il file dei barcode: " << filename << std::endl;
        return false;
    }

    std::string line;
    // Salta le righe di commento (iniziano con '#')
    while(std::getline(file, line) && line.find('#') != std::string::npos);

    // Legge ogni riga del file
    do {
        if(line.empty()) continue;
        std::stringstream ss(line);
        int subject, barcode;
        ss >> subject >> barcode;
        barcode_to_subject_[barcode] = subject;  // Crea la mappatura barcode -> ID soggetto
    } while(std::getline(file, line));

    return true;
}

/**
 * Il file contiene le posizioni reali (groundtruth) dei landmark seguente formato :
 * - landmark_id 
 * - x_pos 
 * - y_pos
 * - std_x
 * - std_y
 * Queste informazioni sono utilizzate dall'EKF per le correzioni basate
 * sulle misurazioni landmark.
 */
bool DataLoader::loadLandmarkGroundtruth(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Errore: Impossibile aprire il file dei landmark: " << filename << std::endl;
        return false;
    }

    std::string line;
    // Salta le righe di commento (che iniziano con '#')
    while (std::getline(file, line) && line.find('#') != std::string::npos);

    // Legge i dati reali dei landmark
    do {
        if (line.empty()) continue;
        std::stringstream ss(line);
        Landmark lm;
        double std_x, std_y;  // Deviazioni standard (non utilizzate attualmente)
        ss >> lm.id >> lm.pos.x() >> lm.pos.y() >> std_x >> std_y;
        landmark_map[lm.id] = lm;  // Memorizza il landmark nella mappa
    } while (std::getline(file, line));

    std::cout << "Caricati " << landmark_map.size() << " landmark." << std::endl;
    return true;
}

/**
 * Il file di odometria contiene le velocità lineari e angolari misurate
 * dai sensori del robot nel formato :
 * - timestamp
 * - velocità_lineare
 * - velocità_angolare
 */
bool DataLoader::loadOdometry(const std::string& filename, int robot_id) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Errore: Impossibile aprire il file di odometria: " << filename << std::endl;
        return false;
    }
    std::string line;
    // Salta le righe di commento
    while (std::getline(file, line) && line.find('#') != std::string::npos);

    // Carica ogni riga di dati odometrici
    do {
        if (line.empty()) continue;
        std::stringstream ss(line);
        OdometryData odom;
        odom.robot_id = robot_id;  // Assegna l'ID del robot
        ss >> odom.timestamp >> odom.v >> odom.w;  // timestamp, velocità lineare, velocità angolare
        event_queue.emplace_back(odom);  // Aggiunge alla coda degli eventi
    } while (std::getline(file, line));

    return true;
}

/**
 * Il file delle misurazioni contiene osservazioni range-bearing nel formato :
 * timestamp barcode_id range bearing
 * Il barcode viene convertito nell'ID reale del soggetto osservato.
 * Il campo is_landmark viene impostato verificando se l'ID corrisponde a un landmark.
 */
bool DataLoader::loadMeasurements(const std::string& filename, int robot_id) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Errore: Impossibile aprire il file delle misure: " << filename << std::endl;
        return false;
    }
    std::string line;
    // Salta le righe di commento
    while (std::getline(file, line) && line.find('#') != std::string::npos);

    // Carica ogni misurazione
    do {
        if (line.empty()) continue;
        std::stringstream ss(line);
        MeasurementData meas;
        meas.observer_id = robot_id;  // Il robot che ha fatto la misurazione
        int barcode;
        ss >> meas.timestamp >> barcode >> meas.range >> meas.bearing;
        
        // Converte il barcode nell'ID reale del soggetto
        auto it = barcode_to_subject_.find(barcode);
        meas.subject_id = (it != barcode_to_subject_.end()) ? it->second : barcode;

        // Determina se è un landmark:
        // - Se landmark_map è disponibile, usa quello (più accurato)
        // - Altrimenti usa convenzione dataset UTIAS: ID > 5 sono landmark
        if (!landmark_map.empty()) {
            meas.is_landmark = (landmark_map.find(meas.subject_id) != landmark_map.end());
        } else {
            // Convenzione UTIAS: robot IDs 1-5, landmark IDs >= 6
            meas.is_landmark = (meas.subject_id > 5);
        }

        event_queue.emplace_back(meas);  // Aggiunge alla coda degli eventi
    } while (std::getline(file, line));

    return true;
}

/**
 * La funzione ordina tutti gli eventi (odometria e misurazioni) per timestamp
 * prima di restituire la coda. Questo garantisce che la riproduzione dei dati
 * avvenga nell'ordine cronologico corretto.
 */
const std::vector<Event>& DataLoader::getEventQueue() const {
    // Ordina la coda prima di restituirla se non è già stato fatto
    // (potrebbe essere più efficiente farlo una sola volta dopo aver caricato tutto)
    std::sort(const_cast<DataLoader*>(this)->event_queue.begin(),
              const_cast<DataLoader*>(this)->event_queue.end(),
              EventComparator());
    return event_queue;
}

/**
 * Questa mappa contiene le posizioni ground truth di tutti i landmark
 * caricati dal dataset, utilizzate dall'EKF per le correzioni.
 */
const std::map<int, Landmark>& DataLoader::getLandmarks() const {
    return landmark_map;
}
