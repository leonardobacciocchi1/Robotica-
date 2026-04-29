/**
 * @file ekf_centralized.cpp
 * @brief Implementazione dell'algoritmo EKF centralizzato
 *
 * Questo file implementa l'algoritmo EKF (Extended Kalman Filter) centralizzato
 * per la localizzazione cooperativa di robot multipli. L'EKF mantiene una stima
 * congiunta delle posizioni di tutti i robot e utilizza:
 * - Odometria per la predizione del movimento
 * - Misurazioni landmark per correzioni assolute
 * - Misurazioni robot-robot per correzioni relative cooperative
 * 
 * @authors Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
 * @date 2025
 */

#include "ekf_centralized.h"
#include <cmath>      
#include <iomanip>  

/**
 * Inizializza le matrici di covarianza e rumore per l'EKF.
 * Ogni robot ha stato 3D: [x, y, θ] dove θ è l'orientamento.
 */
EkfCentralized::EkfCentralized(int num_robots, const Eigen::VectorXd& initial_state) :
    state_(initial_state), num_robots_(num_robots) {

    int state_size = num_robots_ * 3;  // 3 stati per robot (x, y, θ)
    // Inizializza la matrice di covarianza con bassa incertezza iniziale
    P_ = Eigen::MatrixXd::Identity(state_size, state_size) * 0.01;

    // Rumore di processo - valori piccoli per odometria precisa
    // Q rappresenta l'incertezza nel modello di movimento
    Q_ = (Eigen::Matrix3d() << 0.001, 0,     0,      // Varianza in x
                               0,     0.001, 0,      // Varianza in y
                               0,     0,     0.0005).finished();  // Varianza in θ

    // Rumore di misura per landmark - valori più alti per rumore sensore realistico
    R_landmark_ = (Eigen::Matrix2d() << 0.25, 0,     // Varianza range
                                        0,    0.15).finished();  // Varianza bearing

    // Rumore di misura per robot-robot - generalmente più preciso
    R_robot_ = (Eigen::Matrix2d() << 0.2, 0,         // Varianza range
                                   0,   0.1).finished();        // Varianza bearing
}

/**
 * I landmark sono utilizzati per correzioni assolute nell'EKF.
 * Le loro posizioni sono considerate note con precisione.
 */
void EkfCentralized::setLandmarks(const std::map<int, Landmark>& landmarks) {
    landmark_map_ = landmarks;
}

/**
 * Verifica che la dimensione sia corretta prima di aggiornare lo stato.
 * Utile per inizializzazione o reset del sistema.
 */
void EkfCentralized::setState(const Eigen::VectorXd& new_state) {
    if(new_state.size() == state_.size()) {
        state_ = new_state;
    }
}

/**
 * Applica il modello cinematico dell'unicycle per predire il nuovo stato
 * del robot specifico e aggiorna la covarianza corrispondente.
 * Modello: x' = x + v*cos(θ)*dt, y' = y + v*sin(θ)*dt, θ' = θ + w*dt
 */
void EkfCentralized::predict(const OdometryData& odom, double dt) {
    // Valida robot_id
    if(odom.robot_id < 1 || odom.robot_id > num_robots_) {
        return;  // Robot ID fuori range, ignora silenziosamente
    }

    int idx = (odom.robot_id - 1) * 3;  // Indice dello stato del robot
    Eigen::Vector3d state = state_.segment<3>(idx);  // Stato corrente [x, y, θ]

    // Modello cinematico dell'unicycle
    double theta = state[2];
    state[0] += odom.v * cos(theta) * dt;  // Nuova posizione x
    state[1] += odom.v * sin(theta) * dt;  // Nuova posizione y
    state[2] += odom.w * dt;               // Nuovo orientamento θ
    state[2] = normalize_angle(state[2]);   // Normalizza θ in [-π, π]

    state_.segment<3>(idx) = state;  // Aggiorna lo stato

    // Jacobiano del modello di movimento rispetto allo stato
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -odom.v * sin(theta) * dt;  // ∂x'/∂θ
    F(1,2) = odom.v * cos(theta) * dt;   // ∂y'/∂θ

    // Aggiornamento della covarianza: P = F*P*F^T + Q
    P_.block<3,3>(idx, idx) = F * P_.block<3,3>(idx, idx) * F.transpose() + Q_;
}

/**
 * Predizione basata su incremento di posa.
 * Questa versione è preferibile quando si lavora con robot reali dove il driver
 * fornisce già la stima dello spostamento (es. da odometria integrata).
 * Evita problemi di sincronizzazione temporale tra computer e controllore motori.
 *
 * Modello: La posa incrementale viene trasformata dal frame del robot al frame globale
 * usando la rotazione corrente del robot.
 */
void EkfCentralized::predict(const PoseIncrement& pose_inc) {
    // Valida robot_id
    if(pose_inc.robot_id < 1 || pose_inc.robot_id > num_robots_) {
        return;  // Robot ID fuori range, ignora silenziosamente
    }

    int idx = (pose_inc.robot_id - 1) * 3;  // Indice dello stato del robot
    Eigen::Vector3d state = state_.segment<3>(idx);  // Stato corrente [x, y, θ]

    double theta = state[2];  // Orientamento corrente
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    // Trasforma l'incremento dal frame del robot al frame globale
    double dx_global = cos_theta * pose_inc.dx - sin_theta * pose_inc.dy;
    double dy_global = sin_theta * pose_inc.dx + cos_theta * pose_inc.dy;

    // Aggiorna lo stato
    state[0] += dx_global;
    state[1] += dy_global;
    state[2] += pose_inc.dtheta;
    state[2] = normalize_angle(state[2]);

    state_.segment<3>(idx) = state;

    // Jacobiano del modello di movimento rispetto allo stato
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -sin_theta * pose_inc.dx - cos_theta * pose_inc.dy;  // ∂x'/∂θ
    F(1,2) = cos_theta * pose_inc.dx - sin_theta * pose_inc.dy;   // ∂y'/∂θ

    // Aggiornamento della covarianza: P = F*P*F^T + Q
    P_.block<3,3>(idx, idx) = F * P_.block<3,3>(idx, idx) * F.transpose() + Q_;
}

/**
 * Calcola la distanza di Mahalanobis tra la misurazione osservata e quella
 * predetta dal modello. Se la distanza supera la soglia, la misurazione
 * viene considerata un outlier e scartata.
 */
bool EkfCentralized::validateMeasurement(const MeasurementData& meas, double mahalanobis_threshold) {
    if(meas.subject_id <= 0) return false;  // ID non valido

    // Valida observer_id
    if(meas.observer_id < 1 || meas.observer_id > num_robots_) {
        return false;  // Observer ID fuori range
    }

    int observer_idx = (meas.observer_id - 1) * 3;  // Indice robot osservatore
    Eigen::Vector3d obs = state_.segment<3>(observer_idx);  // Stato osservatore

    Eigen::Vector2d z_pred;    // Misurazione predetta
    Eigen::MatrixXd H;         // Jacobiano della funzione di osservazione
    Eigen::Matrix2d R;         // Matrice di rumore

    // Misurazione landmark (usa il flag is_landmark)
    if(meas.is_landmark) {
        auto it = landmark_map_.find(meas.subject_id);
        if(it == landmark_map_.end()) return false;  // Landmark non trovato

        Eigen::Vector2d landmark = it->second.pos;
        double dx = landmark[0] - obs[0];
        double dy = landmark[1] - obs[1];
        double range = sqrt(dx*dx + dy*dy);
        double bearing = atan2(dy, dx) - obs[2];
        bearing = normalize_angle(bearing);
        z_pred << range, bearing;

        // Jacobiano per misurazione landmark
        H = Eigen::MatrixXd::Zero(2, num_robots_*3);
        H.block<2,3>(0, observer_idx) << -dx/range, -dy/range, 0,
                                         dy/(range*range), -dx/(range*range), -1;
        R = R_landmark_;
    }
    // Misurazione robot-robot
    else if(meas.subject_id != meas.observer_id) {
        // Valida subject_id per robot-robot
        if(meas.subject_id < 1 || meas.subject_id > num_robots_) {
            return false;  // Subject ID fuori range
        }

        int subject_idx = (meas.subject_id - 1) * 3;  // Indice robot osservato
        Eigen::Vector3d sub = state_.segment<3>(subject_idx);  // Stato osservato

        double dx = sub[0] - obs[0];
        double dy = sub[1] - obs[1];
        double range = sqrt(dx*dx + dy*dy);
        double bearing = atan2(dy, dx) - obs[2];
        bearing = normalize_angle(bearing);
        z_pred << range, bearing;

        // Jacobiano per misurazione robot-robot (coinvolge entrambi i robot)
        H = Eigen::MatrixXd::Zero(2, num_robots_*3);
        H.block<2,3>(0, observer_idx) << -dx/range, -dy/range, 0,
                                         dy/(range*range), -dx/(range*range), -1;
        H.block<2,3>(0, subject_idx) << dx/range, dy/range, 0,
                                       -dy/(range*range), dx/(range*range), 0;
        R = R_robot_;
    } else {
        return false;  // Robot non può osservare se stesso
    }

    // Calcola la distanza di Mahalanobis
    Eigen::Vector2d z(meas.range, meas.bearing);  // Misurazione osservata
    Eigen::Vector2d dz = z - z_pred;              // Innovazione
    dz[1] = normalize_angle(dz[1]);               // Normalizza differenza angolare

    Eigen::Matrix2d S = H * P_ * H.transpose() + R;  // Matrice di innovazione
    double mahalanobis = sqrt(dz.transpose() * S.inverse() * dz);
    return mahalanobis < mahalanobis_threshold;
}

/**
 * Applica la correzione dell'EKF utilizzando una misurazione range-bearing.
 * Può essere una misurazione landmark (correzione assoluta) o robot-robot
 * (correzione relativa cooperativa). Utilizza l'algoritmo standard EKF.
 */
void EkfCentralized::correct(const MeasurementData& meas) {
    if(!validateMeasurement(meas)) return;  // Scarta misurazioni non valide

    // Doppia verifica observer_id (sicurezza extra)
    if(meas.observer_id < 1 || meas.observer_id > num_robots_) return;

    int state_size = num_robots_ * 3;
    int obs_idx = (meas.observer_id - 1) * 3;  // Indice robot osservatore

    Eigen::VectorXd z_pred(2);  // Misurazione predetta [range, bearing]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state_size);  // Jacobiano
    Eigen::Matrix2d R;  // Matrice di rumore

    // Misurazione landmark (usa il flag is_landmark)
    if(meas.is_landmark) {
        Eigen::Vector3d obs = state_.segment<3>(obs_idx);  // Stato osservatore
        Eigen::Vector2d landmark = landmark_map_[meas.subject_id].pos;  // Posizione landmark

        // Calcola misurazione predetta
        double dx = landmark[0] - obs[0];
        double dy = landmark[1] - obs[1];
        double range = std::sqrt(dx*dx + dy*dy);
        double bearing = std::atan2(dy, dx) - obs[2];
        bearing = normalize_angle(bearing);
        z_pred << range, bearing;

        // Jacobiano della funzione di osservazione landmark
        H.block<2,3>(0, obs_idx) << -dx/range, -dy/range, 0,
                                   dy/(range*range), -dx/(range*range), -1;
        R = R_landmark_;
    }
    // Misurazione robot-robot
    else {
        int sub_idx = (meas.subject_id - 1) * 3;  		   // Indice robot osservato
        Eigen::Vector3d obs = state_.segment<3>(obs_idx);  // Stato osservatore
        Eigen::Vector3d sub = state_.segment<3>(sub_idx);  // Stato osservato

        // Calcola misurazione predetta
        double dx = sub[0] - obs[0];
        double dy = sub[1] - obs[1];
        double range = std::sqrt(dx*dx + dy*dy);
        double bearing = std::atan2(dy, dx) - obs[2];
        bearing = normalize_angle(bearing);
        z_pred << range, bearing;

        // Jacobiano della funzione di osservazione robot-robot
        H.block<2,3>(0, obs_idx) << -dx/range, -dy/range, 0,
                                   dy/(range*range), -dx/(range*range), -1;
        H.block<2,3>(0, sub_idx) << dx/range, dy/range, 0,
                                   -dy/(range*range), dx/(range*range), 0;
        R = R_robot_;
    }

    // Algoritmo standard EKF
    Eigen::Vector2d z(meas.range, meas.bearing);  		  // Misurazione osservata
    Eigen::Vector2d dz = z - z_pred;              	      // Innovazione
    dz[1] = normalize_angle(dz[1]);               		  // Normalizza differenza angolare

    // Calcola guadagno di Kalman
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;       // Matrice di innovazione
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse(); // Guadagno di Kalman

    // Aggiorna stato
    state_ += K * dz;
    // Normalizza tutti gli angoli dei robot
    for(int r = 0; r < num_robots_; ++r) {
        state_[3*r + 2] = normalize_angle(state_[3*r + 2]);
    }

    // Aggiorna covarianza (forma numericamente stabile di Joseph)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

    last_correction_time_ = meas.timestamp;  // Memorizza timestamp correzione
}

/**
 * Correzione dell'EKF con vettore di misurazioni multiple.
 * Questa versione permette di processare più misurazioni in un unico ciclo,
 * costruendo un vettore di innovazione stacked e una matrice H completa.
 * Utile per contesti più generali rispetto al dataset UTIAS dove le misurazioni
 * possono arrivare in batch.
 */
void EkfCentralized::correct(const std::vector<MeasurementData>& measurements) {
    if(measurements.empty()) return;

    // Filtra misurazioni valide
    std::vector<MeasurementData> valid_meas;
    for(const auto& meas : measurements) {
        if(validateMeasurement(meas)) {
            valid_meas.push_back(meas);
        }
    }

    if(valid_meas.empty()) return;

    int state_size = num_robots_ * 3;
    int num_meas = valid_meas.size();
    int z_size = num_meas * 2;  // Ogni misurazione ha 2 dimensioni (range, bearing)

    // Inizializza vettori e matrici per batch update
    Eigen::VectorXd z(z_size);           // Vettore misurazioni osservate
    Eigen::VectorXd z_pred(z_size);      // Vettore misurazioni predette
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(z_size, state_size);  // Jacobiano completo
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(z_size, z_size);      // Matrice rumore completa

    // Costruisce il vettore di misurazioni e la matrice H
    for(int i = 0; i < num_meas; ++i) {
        const auto& meas = valid_meas[i];
        int z_idx = i * 2;
        int obs_idx = (meas.observer_id - 1) * 3;

        // Misurazione osservata
        z(z_idx) = meas.range;
        z(z_idx + 1) = meas.bearing;

        // Misurazione landmark
        if(meas.is_landmark) {
            Eigen::Vector3d obs = state_.segment<3>(obs_idx);
            Eigen::Vector2d landmark = landmark_map_[meas.subject_id].pos;

            double dx = landmark[0] - obs[0];
            double dy = landmark[1] - obs[1];
            double range = std::sqrt(dx*dx + dy*dy);
            double bearing = std::atan2(dy, dx) - obs[2];
            bearing = normalize_angle(bearing);

            z_pred(z_idx) = range;
            z_pred(z_idx + 1) = bearing;

            H.block<2,3>(z_idx, obs_idx) << -dx/range, -dy/range, 0,
                                            dy/(range*range), -dx/(range*range), -1;
            R.block<2,2>(z_idx, z_idx) = R_landmark_;
        }
        // Misurazione robot-robot
        else {
            int sub_idx = (meas.subject_id - 1) * 3;
            Eigen::Vector3d obs = state_.segment<3>(obs_idx);
            Eigen::Vector3d sub = state_.segment<3>(sub_idx);

            double dx = sub[0] - obs[0];
            double dy = sub[1] - obs[1];
            double range = std::sqrt(dx*dx + dy*dy);
            double bearing = std::atan2(dy, dx) - obs[2];
            bearing = normalize_angle(bearing);

            z_pred(z_idx) = range;
            z_pred(z_idx + 1) = bearing;

            H.block<2,3>(z_idx, obs_idx) << -dx/range, -dy/range, 0,
                                            dy/(range*range), -dx/(range*range), -1;
            H.block<2,3>(z_idx, sub_idx) << dx/range, dy/range, 0,
                                            -dy/(range*range), dx/(range*range), 0;
            R.block<2,2>(z_idx, z_idx) = R_robot_;
        }
    }

    // Calcola innovazione
    Eigen::VectorXd dz = z - z_pred;
    for(int i = 0; i < num_meas; ++i) {
        dz(i*2 + 1) = normalize_angle(dz(i*2 + 1));  // Normalizza bearing
    }

    // Algoritmo standard EKF con batch update
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // Aggiorna stato
    state_ += K * dz;
    for(int r = 0; r < num_robots_; ++r) {
        state_[3*r + 2] = normalize_angle(state_[3*r + 2]);
    }

    // Aggiorna covarianza (forma di Joseph)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

    // Aggiorna timestamp con l'ultima misurazione
    if(!valid_meas.empty()) {
        last_correction_time_ = valid_meas.back().timestamp;
    }
}

/**
 * Stampa le posizioni e orientamenti stimati di tutti i robot
 * in formato leggibile.
 */
void EkfCentralized::printState(rclcpp::Logger logger, double timestamp) const {
    std::stringstream ss;
    ss << "\nTime " << std::fixed << std::setprecision(3) << timestamp;
    ss << "\nCurrent state:";
    for(int i = 0; i < num_robots_; ++i) {
        ss << "\nRobot " << i+1 << ": "
           << std::fixed << std::setprecision(3)
           << "x=" << state_(3*i)      // Posizione x
           << " y=" << state_(3*i+1)   // Posizione y
           << " θ=" << state_(3*i+2);  // Orientamento θ
    }
    RCLCPP_INFO(logger, "%s", ss.str().c_str());
}

/**
 * Stampa gli elementi diagonali della matrice di covarianza,
 * che rappresentano le varianze delle stime per ogni robot.
 */
void EkfCentralized::printUncertainty(rclcpp::Logger logger) const {
    std::stringstream ss;
    ss << "\nUncertainty (diagonal elements):";
    for(int i = 0; i < num_robots_; ++i) {
        ss << "\nRobot " << i+1 << ": "
           << std::fixed << std::setprecision(5)
           << "σ²_x=" << P_(3*i, 3*i)      // Varianza posizione x
           << " σ²_y=" << P_(3*i+1, 3*i+1) // Varianza posizione y
           << " σ²_θ=" << P_(3*i+2, 3*i+2); // Varianza orientamento θ
    }
    RCLCPP_DEBUG(logger, "%s", ss.str().c_str());
}

/**
 * Funzione di utilità per mantenere gli angoli in un intervallo
 * standard, evitando problemi di discontinuità angolare.
 */
double EkfCentralized::normalize_angle(double angle) {
    while(angle > M_PI) angle -= 2*M_PI;   // Riduce angoli > π
    while(angle < -M_PI) angle += 2*M_PI;  // Aumenta angoli < -π
    return angle;
}

/**
 * Registra uno snapshot delle covarianze correnti per monitoraggio.
 * Estrae le deviazioni standard (sigma) dalla matrice di covarianza
 * per ogni robot e le salva insieme al timestamp.
 */
void EkfCentralized::recordCovarianceSnapshot(double timestamp) {
    CovarianceSnapshot snapshot;
    snapshot.timestamp = timestamp;
    snapshot.sigma_x.resize(num_robots_);
    snapshot.sigma_y.resize(num_robots_);
    snapshot.sigma_theta.resize(num_robots_);

    for(int i = 0; i < num_robots_; ++i) {
        int idx = i * 3;
        snapshot.sigma_x[i] = std::sqrt(P_(idx, idx));
        snapshot.sigma_y[i] = std::sqrt(P_(idx+1, idx+1));
        snapshot.sigma_theta[i] = std::sqrt(P_(idx+2, idx+2));
    }

    covariance_history_.push_back(snapshot);
}

/**
 * Salva lo storico delle covarianze in un file CSV.
 * Formato: timestamp, robot_id, sigma_x, sigma_y, sigma_theta
 * Utile per analisi post-processing e monitoraggio della stabilità del filtro.
 */
void EkfCentralized::saveCovarianceHistory(const std::string& filename) const {
    std::ofstream file(filename);
    if(!file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file " << filename << std::endl;
        return;
    }

    // Header CSV
    file << "timestamp,robot_id,sigma_x,sigma_y,sigma_theta\n";

    // Dati
    for(const auto& snapshot : covariance_history_) {
        for(int i = 0; i < num_robots_; ++i) {
            file << std::fixed << std::setprecision(6)
                 << snapshot.timestamp << ","
                 << (i + 1) << ","
                 << snapshot.sigma_x[i] << ","
                 << snapshot.sigma_y[i] << ","
                 << snapshot.sigma_theta[i] << "\n";
        }
    }

    file.close();
    std::cout << "Storico covarianze salvato in: " << filename << std::endl;
}
