#ifndef JERRO_DRIVERS_OSCILLATION_DETECTOR_HPP
#define JERRO_DRIVERS_OSCILLATION_DETECTOR_HPP

#include <vector>
#include <cmath>

// Détecteur d'oscillations pour auto-tuning Ziegler-Nichols
// Détecte les pics et vallées (extrema locaux) au lieu des passages par zéro
// Permet de détecter les oscillations même avec un offset DC non nul
struct OscillationDetector {
    // Configuration
    int min_cycles_required = 4;
    float period_tolerance = 0.30f;         // Tolérance de 30% sur la période
    float min_extremum_amplitude = 20.0f;   // Amplitude minimale pour détecter un extremum (ticks)
    float min_period = 0.2f;                // Période minimale acceptée (secondes) - évite bruit HF

    // État
    std::vector<float> extremum_times;    // Temps des pics/vallées
    std::vector<float> extremum_values;   // Valeurs des pics/vallées
    float last_error = 0;
    float second_last_error = 0;          // Pour détecter tendance (besoin de 3 points)
    float filtered_error = 0;             // Erreur filtrée (pour réduire bruit)
    float time_accumulator = 0;
    bool last_was_peak = false;           // Pour alterner pics/vallées
    bool has_extremum = false;            // Pour initialisation

    bool detectOscillation(float error, float dt, float& Tu_out) {
        time_accumulator += dt;

        // Besoin de 3 points pour détecter un extremum local
        if (!has_extremum && second_last_error != 0) {
            // Détecte pic (maximum local): erreur monte puis descend
            if (second_last_error < last_error && last_error > error) {
                extremum_times.push_back(time_accumulator - dt);
                extremum_values.push_back(last_error);
                last_was_peak = true;
                has_extremum = true;
            }
            // Détecte vallée (minimum local): erreur descend puis monte
            else if (second_last_error > last_error && last_error < error) {
                extremum_times.push_back(time_accumulator - dt);
                extremum_values.push_back(last_error);
                last_was_peak = false;
                has_extremum = true;
            }
        }
        // Détecte extrema suivants (en alternance pic/vallée)
        else if (has_extremum) {
            bool is_peak = (second_last_error < last_error && last_error > error);
            bool is_valley = (second_last_error > last_error && last_error < error);

            // Accepte pic après vallée, ou vallée après pic
            if ((is_peak && !last_was_peak) || (is_valley && last_was_peak)) {
                // Vérifie l'amplitude par rapport au dernier extremum
                float amplitude = std::abs(last_error - extremum_values.back());

                // Filtre: rejette les extrema avec amplitude trop faible (bruit)
                if (amplitude >= min_extremum_amplitude) {
                    extremum_times.push_back(time_accumulator - dt);
                    extremum_values.push_back(last_error);
                    last_was_peak = is_peak;

                    // Besoin d'au moins 9 extrema pour 4 cycles complets
                    // 1 cycle = pic → vallée → pic (3 extrema)
                    // 4 cycles = 9 extrema
                    if (extremum_times.size() >= 9) {
                        if (isOscillationSustained(Tu_out)) {
                            return true;
                        }
                    }
                }
            }
        }

        // Mise à jour historique
        second_last_error = last_error;
        last_error = error;
        return false;
    }

    bool isOscillationSustained(float& Tu_out) {
        // Calcule les périodes complètes entre extrema de même type
        // (pic à pic ou vallée à vallée)
        std::vector<float> periods;
        for (size_t i = 2; i < extremum_times.size(); i += 2) {
            periods.push_back(extremum_times[i] - extremum_times[i-2]);
        }

        if (periods.size() < 4) return false;

        // Prend les 4 dernières périodes
        std::vector<float> recent_periods(
            periods.end() - 4,
            periods.end()
        );

        // Calcule moyenne et vérifie cohérence
        float mean_period = 0;
        for (float p : recent_periods) {
            mean_period += p;
        }
        mean_period /= recent_periods.size();

        // Filtre 1: Rejette si période moyenne trop courte (bruit HF)
        if (mean_period < min_period) {
            return false;  // Période trop courte = bruit haute fréquence
        }

        // Filtre 2: Vérifie que toutes les périodes sont cohérentes
        for (float p : recent_periods) {
            if (std::abs(p - mean_period) > mean_period * period_tolerance) {
                return false;  // Trop de variation
            }
        }

        Tu_out = mean_period;
        return true;
    }

    void reset() {
        extremum_times.clear();
        extremum_values.clear();
        last_error = 0;
        second_last_error = 0;
        time_accumulator = 0;
        last_was_peak = false;
        has_extremum = false;
    }
};

#endif  // JERRO_DRIVERS_OSCILLATION_DETECTOR_HPP
