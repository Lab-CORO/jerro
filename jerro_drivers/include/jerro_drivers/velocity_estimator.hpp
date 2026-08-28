#ifndef JERRO_DRIVERS_VELOCITY_ESTIMATOR_HPP
#define JERRO_DRIVERS_VELOCITY_ESTIMATOR_HPP

#include <cmath>
#include <cstddef>
#include <deque>

// Estimateur de vitesse a fenetre adaptative.
//
// Le calcul naif (delta_ticks / dt) sur la periode fixe de la boucle (20 ms a
// 50 Hz) est correct a vitesse elevee mais devient inutilisable a basse vitesse :
//   - a 200 ticks/s -> ~4 ticks par cycle, resolution correcte
//   - a  20 ticks/s -> ~0.4 tick par cycle, la mesure saute entre 0 et 50 ticks/s
// Ce bruit de quantification est ensuite amplifie par le terme derive.
//
// Principe : on conserve un historique court de (compteur, temps) et on calcule
// la vitesse sur la fenetre la plus COURTE contenant au moins min_ticks ticks.
// A vitesse elevee la fenetre reste courte (peu de retard de phase) ; a basse
// vitesse elle s'allonge automatiquement jusqu'a max_window pour recuperer de la
// resolution.
class VelocityEstimator {
public:
    float min_ticks = 4.0f;     // Nombre de ticks vise dans la fenetre
    float min_window = 0.015f;  // Fenetre minimale (s) - evite la division par ~0
    float max_window = 0.2f;    // Fenetre maximale (s) - borne le retard de phase

    VelocityEstimator() = default;

    // count : compteur encodeur cumule
    // t     : temps monotone en secondes
    float update(int count, float t) {
        samples_.push_back(Sample{count, t});

        // Purge de l'historique trop ancien (on garde toujours >= 2 echantillons)
        while (samples_.size() > 2 && (t - samples_.front().t) > max_window) {
            samples_.pop_front();
        }

        if (samples_.size() < 2) {
            return last_velocity_;
        }

        // Recherche, du plus recent vers le plus ancien, de la premiere fenetre
        // qui satisfait a la fois min_window et min_ticks.
        for (std::size_t i = samples_.size() - 1; i-- > 0;) {
            float span = t - samples_[i].t;
            if (span < min_window) continue;

            float dticks = static_cast<float>(count - samples_[i].count);
            if (std::abs(dticks) >= min_ticks) {
                last_velocity_ = dticks / span;
                return last_velocity_;
            }
        }

        // Aucune fenetre n'atteint min_ticks : on utilise tout l'historique
        // disponible. Si le moteur est reellement a l'arret, cela donne bien 0.
        const Sample& oldest = samples_.front();
        float span = t - oldest.t;
        if (span >= min_window) {
            last_velocity_ = static_cast<float>(count - oldest.count) / span;
        }
        return last_velocity_;
    }

    void reset() {
        samples_.clear();
        last_velocity_ = 0.0f;
    }

    float getValue() const { return last_velocity_; }

private:
    struct Sample {
        int count;
        float t;
    };

    std::deque<Sample> samples_;
    float last_velocity_ = 0.0f;
};

#endif  // JERRO_DRIVERS_VELOCITY_ESTIMATOR_HPP
