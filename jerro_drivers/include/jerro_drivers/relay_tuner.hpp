#ifndef JERRO_DRIVERS_RELAY_TUNER_HPP
#define JERRO_DRIVERS_RELAY_TUNER_HPP

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// Auto-tuning par retour a relais (Astrom-Hagglund).
//
// Au lieu de chercher Ku en balayant Kp jusqu'a ce qu'une oscillation apparaisse
// par hasard (des centaines de secondes, et rien ne garantit qu'elle apparaisse),
// on FORCE l'oscillation avec une commande en relais et on en deduit Ku
// analytiquement :
//
//   PWM = bias + d   si  vitesse < consigne - h
//   PWM = bias - d   si  vitesse > consigne + h
//
//   a  = amplitude (demi crete-a-crete) de l'oscillation de vitesse
//   Tu = periode moyenne du cycle limite
//   Ku = 4d / (pi * sqrt(a^2 - h^2))      (h = hysteresis ; 4d/(pi*a) si h = 0)
//
// Le moteur n'oscille pas autour de PWM = 0 a consigne non nulle : le relais doit
// etre centre sur le PWM qui maintient la consigne, d'ou la phase de recherche du
// biais.
//
// Utilisation : start(), puis update(vitesse, dt) A CHAQUE NOUVEL ECHANTILLON
// D'ENCODEUR (et non a cadence libre - echantillonner plus vite que l'encodeur ne
// publie produit de l'aliasing et un bruit de mesure artificiel). La valeur
// retournee est le PWM a appliquer. On s'arrete quand state() vaut DONE (Ku/Tu
// renseignes) ou FAILED (message renseigne).
class RelayTuner {
public:
    enum class State {
        BIAS_SEARCH,    // Recherche integrale du PWM qui maintient la consigne
        NOISE_MEASURE,  // Mesure du bruit a PWM fige -> dimensionne l'hysteresis
        RELAY,          // Cycle limite force
        DONE,
        FAILED
    };

    // --- Configuration ---
    float relay_amplitude = 40.0f;   // d : amplitude PWM du relais
    float min_hysteresis = 10.0f;    // h plancher (ticks/s)
    int min_cycles = 5;              // Cycles coherents requis pour converger
    int skip_cycles = 2;             // Cycles initiaux ignores (transitoire)
    float period_tolerance = 0.30f;  // Coherence exigee sur les periodes
    float amplitude_tolerance = 0.40f;
    float min_period = 0.05f;        // Rejette le bruit haute frequence
    float bias_rate = 60.0f;         // PWM/s a erreur relative pleine echelle
    float bias_max = 220.0f;         // Saturation de la recherche de biais
    float bias_tolerance = 0.05f;    // |erreur|/consigne acceptee pour "converge"
    float bias_settle_time = 1.5f;   // Duree de stabilite exigee avant de figer
    float noise_duration = 1.0f;     // Duree de la mesure de bruit (s)

    // Plafond de l'hysteresis, en fraction de la consigne. Sans lui, une mesure
    // bruitee donne h = 3*sigma superieur a la consigne elle-meme : le relais ne
    // peut alors JAMAIS commuter et l'identification tourne dans le vide jusqu'au
    // timeout, sans rien mesurer.
    float hysteresis_max_ratio = 0.25f;

    // Delai maximal sans commutation avant d'abandonner. Evite de consommer tout
    // le budget de temps quand le relais est manifestement bloque.
    float max_switch_gap = 5.0f;

    // --- Resultats ---
    float Ku = 0.0f;
    float Tu = 0.0f;
    float amplitude = 0.0f;
    float hysteresis = 0.0f;
    float bias = 0.0f;
    std::string message;
    std::string note;  // Avertissement non bloquant, propage dans les diagnostics

    void start(float target_velocity) {
        target_ = target_velocity;
        state_ = State::BIAS_SEARCH;
        t_ = 0.0f;
        phase_start_ = 0.0f;
        bias = 0.0f;
        effective_d_ = relay_amplitude;
        Ku = Tu = amplitude = hysteresis = 0.0f;
        message.clear();
        note.clear();
        periods_.clear();
        amplitudes_.clear();
        noise_sum_ = noise_sum_sq_ = 0.0f;
        noise_n_ = 0;
        in_tolerance_since_ = -1.0f;
        have_last_switch_ = false;
        last_switch_t_ = 0.0f;
        last_cycle_t_ = 0.0f;
        relay_up_ = true;
        cyc_max_ = cyc_min_ = 0.0f;
        last_velocity_ = 0.0f;
    }

    // A appeler une fois par echantillon d'encodeur. Retourne le PWM a appliquer.
    float update(float velocity, float dt) {
        t_ += dt;
        last_velocity_ = velocity;

        switch (state_) {
            case State::BIAS_SEARCH:
                return updateBiasSearch(velocity, dt);
            case State::NOISE_MEASURE:
                return updateNoiseMeasure(velocity);
            case State::RELAY:
                return updateRelay(velocity);
            default:
                return 0.0f;
        }
    }

    State state() const { return state_; }

    const char* stateName() const {
        switch (state_) {
            case State::BIAS_SEARCH: return "recherche du biais";
            case State::NOISE_MEASURE: return "mesure du bruit";
            case State::RELAY: return "cycle limite";
            case State::DONE: return "termine";
            default: return "echec";
        }
    }

    int cyclesCollected() const { return static_cast<int>(periods_.size()); }

    // Appele par le noeud quand le budget de temps global est epuise, afin que le
    // message d'echec decrive ce qui a reellement ete observe.
    void timeout() {
        if (state_ == State::DONE || state_ == State::FAILED) return;

        switch (state_) {
            case State::BIAS_SEARCH:
                message = "timeout pendant la recherche du biais (PWM=" + fmt(bias) +
                          ", vitesse " + fmt(last_velocity_) + " pour une consigne de " +
                          fmt(target_) + " ticks/s)";
                break;
            case State::NOISE_MEASURE:
                message = "timeout pendant la mesure de bruit";
                break;
            default:
                message = "timeout : " + std::to_string(periods_.size()) +
                          " cycle(s) mesure(s) sur " +
                          std::to_string(skip_cycles + min_cycles) +
                          " requis, periodes incoherentes (bias=" + fmt(bias) +
                          ", h=" + fmt(hysteresis) + ")";
                break;
        }
        appendNote();
        state_ = State::FAILED;
    }

private:
    State state_ = State::FAILED;
    float target_ = 0.0f;
    float t_ = 0.0f;
    float phase_start_ = 0.0f;
    float effective_d_ = 0.0f;  // d reellement applique (voir freezeBias)
    float last_velocity_ = 0.0f;

    float in_tolerance_since_ = -1.0f;

    float noise_sum_ = 0.0f;
    float noise_sum_sq_ = 0.0f;
    int noise_n_ = 0;

    bool relay_up_ = true;
    bool have_last_switch_ = false;
    float last_switch_t_ = 0.0f;
    float last_cycle_t_ = 0.0f;
    float cyc_max_ = 0.0f;
    float cyc_min_ = 0.0f;

    std::vector<float> periods_;
    std::vector<float> amplitudes_;

    static std::string fmt(float v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.1f", v);
        return std::string(buf);
    }

    void appendNote() {
        if (!note.empty()) message += " [" + note + "]";
    }

    void fail(const std::string& why) {
        message = why;
        appendNote();
        state_ = State::FAILED;
    }

    // Recherche integrale du biais.
    //
    // L'ancienne rampe s'arretait au PREMIER franchissement de la consigne. Comme
    // la mesure de vitesse est en retard sur la realite (inertie + fenetre
    // d'estimation), le PWM etait alors deja bien au-dela du necessaire et le
    // moteur continuait d'accelerer : on a observe un biais donnant 374 ticks/s
    // pour une consigne de 200. Ici le biais converge sur l'erreur, donc sans
    // depassement, et la stabilite exigee avant de figer tient lieu de temps
    // d'etablissement.
    float updateBiasSearch(float velocity, float dt) {
        float rel_err = (target_ - velocity) / target_;
        if (rel_err > 1.0f) rel_err = 1.0f;
        if (rel_err < -1.0f) rel_err = -1.0f;

        bias += bias_rate * rel_err * dt;
        if (bias < 0.0f) bias = 0.0f;

        if (bias >= bias_max) {
            bias = bias_max;
            fail("impossible d'atteindre la consigne de " + fmt(target_) +
                 " ticks/s (PWM sature a " + fmt(bias_max) +
                 ", vitesse mesuree " + fmt(velocity) + ")");
            return 0.0f;
        }

        if (std::abs(rel_err) <= bias_tolerance) {
            if (in_tolerance_since_ < 0.0f) in_tolerance_since_ = t_;
            if (t_ - in_tolerance_since_ >= bias_settle_time) {
                freezeBias();
            }
        } else {
            in_tolerance_since_ = -1.0f;  // sortie de tolerance : on repart a zero
        }

        return bias;
    }

    void freezeBias() {
        // Le relais doit rester SYMETRIQUE : si bias + d depasse 255 ou si
        // bias - d passe sous 0, une des deux alternances est ecretee et le Ku
        // calcule est faux. On reduit d plutot que de laisser saturer.
        effective_d_ = std::min(relay_amplitude, std::min(bias, 255.0f - bias));

        if (effective_d_ < 5.0f) {
            fail("amplitude de relais exploitable trop faible (d=" + fmt(effective_d_) +
                 " pour un biais de " + fmt(bias) +
                 " PWM) - choisir une consigne plus eloignee des extremes");
            return;
        }

        state_ = State::NOISE_MEASURE;
        phase_start_ = t_;
        noise_sum_ = noise_sum_sq_ = 0.0f;
        noise_n_ = 0;
    }

    float updateNoiseMeasure(float velocity) {
        noise_sum_ += velocity;
        noise_sum_sq_ += velocity * velocity;
        noise_n_++;

        if (t_ - phase_start_ >= noise_duration && noise_n_ > 1) {
            float mean = noise_sum_ / static_cast<float>(noise_n_);
            float var = noise_sum_sq_ / static_cast<float>(noise_n_) - mean * mean;
            float sigma = var > 0.0f ? std::sqrt(var) : 0.0f;

            // Sans hysteresis, la quantification de l'encodeur fait commuter le
            // relais sur du bruit et la periode mesuree n'a aucun sens.
            float h = std::max(3.0f * sigma, min_hysteresis);

            // ... mais une hysteresis superieure a la consigne rend la commutation
            // impossible. On plafonne, et on signale que la mesure est douteuse.
            float h_max = hysteresis_max_ratio * target_;
            if (h > h_max) {
                note = "bruit de mesure eleve: sigma=" + fmt(sigma) +
                       " ticks/s, h=3*sigma=" + fmt(h) + " plafonne a " + fmt(h_max);
                h = h_max;
            }
            if (h < min_hysteresis) h = min_hysteresis;

            hysteresis = h;
            state_ = State::RELAY;
            phase_start_ = t_;
            last_switch_t_ = t_;
            have_last_switch_ = false;
            relay_up_ = true;
            cyc_max_ = cyc_min_ = velocity;
        }
        return bias;
    }

    float updateRelay(float velocity) {
        cyc_max_ = std::max(cyc_max_, velocity);
        cyc_min_ = std::min(cyc_min_, velocity);

        bool prev_up = relay_up_;
        if (velocity > target_ + hysteresis) {
            relay_up_ = false;
        } else if (velocity < target_ - hysteresis) {
            relay_up_ = true;
        }

        if (prev_up != relay_up_) {
            last_switch_t_ = t_;
        }

        // Une transition montee -> descente delimite un cycle complet.
        if (prev_up && !relay_up_) {
            if (have_last_switch_) {
                float period = t_ - last_cycle_t_;
                if (period >= min_period) {
                    periods_.push_back(period);
                    amplitudes_.push_back((cyc_max_ - cyc_min_) * 0.5f);
                }
            }
            last_cycle_t_ = t_;
            have_last_switch_ = true;
            cyc_max_ = cyc_min_ = velocity;

            if (static_cast<int>(periods_.size()) >= skip_cycles + min_cycles) {
                tryConverge();
            }
        }

        // Abandon anticipe : le relais est bloque d'un cote.
        if (t_ - last_switch_t_ > max_switch_gap) {
            fail("relais bloque: aucune commutation depuis " + fmt(max_switch_gap) +
                 " s (vitesse " + fmt(velocity) + ", consigne " + fmt(target_) +
                 ", h=" + fmt(hysteresis) + ", seuils " +
                 fmt(target_ - hysteresis) + "/" + fmt(target_ + hysteresis) +
                 ") - augmenter relay_amplitude ou reduire l'hysteresis");
            return 0.0f;
        }

        return relay_up_ ? (bias + effective_d_) : (bias - effective_d_);
    }

    void tryConverge() {
        // On ignore les premiers cycles (transitoire) et on exige que les
        // min_cycles derniers soient coherents en periode ET en amplitude.
        const std::size_t n = static_cast<std::size_t>(min_cycles);
        if (periods_.size() < n) return;

        std::vector<float> p(periods_.end() - n, periods_.end());
        std::vector<float> a(amplitudes_.end() - n, amplitudes_.end());

        float mean_p = 0.0f, mean_a = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            mean_p += p[i];
            mean_a += a[i];
        }
        mean_p /= static_cast<float>(n);
        mean_a /= static_cast<float>(n);

        if (mean_p < min_period || mean_a <= 0.0f) return;

        for (std::size_t i = 0; i < n; ++i) {
            if (std::abs(p[i] - mean_p) > mean_p * period_tolerance) return;
            if (std::abs(a[i] - mean_a) > mean_a * amplitude_tolerance) return;
        }

        // Formule du relais avec hysteresis. Si l'amplitude ne depasse pas
        // l'hysteresis, le cycle limite n'est pas exploitable.
        float denom = mean_a * mean_a - hysteresis * hysteresis;
        if (denom <= 0.0f) {
            return;  // amplitude trop faible face a h : on continue a mesurer
        }

        Tu = mean_p;
        amplitude = mean_a;
        Ku = (4.0f * effective_d_) / (static_cast<float>(M_PI) * std::sqrt(denom));
        state_ = State::DONE;
    }

};

#endif  // JERRO_DRIVERS_RELAY_TUNER_HPP
