#ifndef JERRO_DRIVERS_PID_CONTROLLER_HPP
#define JERRO_DRIVERS_PID_CONTROLLER_HPP

#include <cmath>
#include <iostream>
#include <string>

#include "jerro_drivers/exponential_filter.hpp"

// Variantes de formules de calcul des gains a partir de Ku/Tu.
// Le Ziegler-Nichols "classique" vise une decroissance au quart d'amplitude,
// c'est-a-dire une reponse volontairement oscillante (~25% de depassement).
// Pour une boucle de vitesse sur robot differentiel, NO_OVERSHOOT est
// generalement preferable.
enum class ZNVariant {
    CLASSIC,
    NO_OVERSHOOT,
    PESSEN,
    TYREUS_LUYBEN
};

inline ZNVariant znVariantFromString(const std::string& name) {
    if (name == "classic") return ZNVariant::CLASSIC;
    if (name == "pessen") return ZNVariant::PESSEN;
    if (name == "tyreus_luyben") return ZNVariant::TYREUS_LUYBEN;
    return ZNVariant::NO_OVERSHOOT;
}

inline const char* znVariantName(ZNVariant v) {
    switch (v) {
        case ZNVariant::CLASSIC: return "classic";
        case ZNVariant::PESSEN: return "pessen";
        case ZNVariant::TYREUS_LUYBEN: return "tyreus_luyben";
        default: return "no_overshoot";
    }
}

class PIDController {
public:
    // Gains
    float Kp = 0.1f;
    float Ki = 0.0f;
    float Kd = 0.0f;

    // State
    float error = 0;
    float raw_error = 0;       // Erreur brute avant deadzone (pour detection oscillation)
    float last_error = 0;
    float integral = 0;
    float derivative = 0;

    // Limits
    float integral_max = 100.0f;
    float output_max = 200.0f;
    float deadband_pwm = 15.0f;
    float error_deadzone = 5.0f;

    // Largeur de la zone de transition sur laquelle la compensation de zone morte
    // monte progressivement de 0 a deadband_pwm. Evite le saut discontinu de
    // +/-deadband_pwm au passage par zero, qui transforme le PID en controleur a
    // relais et genere un cycle limite a basse vitesse.
    float deadband_blend = 10.0f;

    PIDController() : derivative_filter_(0.15f) {}

    void setDerivativeFilterAlpha(float alpha) {
        derivative_filter_ = ExponentialFilter(alpha);
    }

    float compute(float setpoint, float measured, float dt) {
        raw_error = setpoint - measured;  // Stocker erreur brute AVANT deadzone
        error = raw_error;                // Copie pour traitement

        if (std::abs(error) < error_deadzone) {
            error = 0;  // Deadzone appliquee seulement a 'error', pas 'raw_error'
        }

        integral += error * dt;
        if (integral > integral_max) integral = integral_max;
        if (integral < -integral_max) integral = -integral_max;

        // Derivee sur la MESURE et non sur l'erreur.
        // Supprime le pic de derivee au changement de consigne, ainsi que le pic
        // parasite provoque par le passage brutal de 'error' a 0 lorsque l'erreur
        // entre dans error_deadzone.
        float d_measured = 0.0f;
        if (derivative_initialized_) {
            d_measured = (measured - last_measured_) / dt;
        } else {
            derivative_initialized_ = true;
        }
        last_measured_ = measured;

        // Filtrage passe-bas du terme derive : sans cela Kd amplifie directement
        // le bruit de quantification de l'encodeur.
        derivative = -derivative_filter_.update(d_measured);

        float output = Kp * error + Ki * integral + Kd * derivative;

        if (output > output_max) output = output_max;
        if (output < -output_max) output = -output_max;

        // Compensation zone morte, continue au passage par zero
        float mag = std::abs(output);
        if (mag > 1e-6f) {
            float comp = deadband_pwm;
            if (deadband_blend > 1e-6f && mag < deadband_blend) {
                comp = deadband_pwm * (mag / deadband_blend);
            }
            mag += comp;
            if (mag > 255.0f) mag = 255.0f;
            output = std::copysign(mag, output);
        } else {
            output = 0.0f;
        }

        last_error = error;
        return output;
    }

    void reset() {
        error = 0;
        last_error = 0;
        integral = 0;
        derivative = 0;
        last_measured_ = 0;
        derivative_initialized_ = false;
        derivative_filter_.reset();
    }

    void setDeadband(float pwm) {
        deadband_pwm = pwm;
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void calculateZieglerNicholsGains(float Ku, float Tu,
                                      ZNVariant variant = ZNVariant::NO_OVERSHOOT) {
        if (Tu <= 1e-6f || Ku <= 0.0f) {
            std::cerr << "AVERTISSEMENT: Ku/Tu invalides, gains inchanges!" << std::endl;
            return;
        }

        float new_Kp = 0.0f, new_Ki = 0.0f, new_Kd = 0.0f;

        switch (variant) {
            case ZNVariant::CLASSIC:
                new_Kp = 0.6f * Ku;
                new_Ki = 1.2f * Ku / Tu;
                new_Kd = 0.075f * Ku * Tu;
                break;
            case ZNVariant::PESSEN:
                new_Kp = 0.7f * Ku;
                new_Ki = 1.75f * Ku / Tu;
                new_Kd = 0.105f * Ku * Tu;
                break;
            case ZNVariant::TYREUS_LUYBEN:
                new_Kp = 0.45f * Ku;
                new_Ki = 0.45f * Ku / (2.2f * Tu);
                new_Kd = 0.45f * Ku * Tu / 6.3f;
                break;
            case ZNVariant::NO_OVERSHOOT:
            default:
                new_Kp = 0.2f * Ku;
                new_Ki = 0.4f * Ku / Tu;
                new_Kd = 0.066f * Ku * Tu;
                break;
        }

        // Validation des gains (affiche avertissements si hors plages)
        if (new_Kp < 0 || new_Kp > 5.0f) {
            std::cerr << "AVERTISSEMENT: Kp hors plage attendue!" << std::endl;
        }
        if (new_Ki < 0 || new_Ki > 20.0f) {
            std::cerr << "AVERTISSEMENT: Ki hors plage attendue!" << std::endl;
        }
        if (new_Kd < 0 || new_Kd > 1.0f) {
            std::cerr << "AVERTISSEMENT: Kd hors plage attendue!" << std::endl;
        }

        setGains(new_Kp, new_Ki, new_Kd);
    }

private:
    ExponentialFilter derivative_filter_;
    float last_measured_ = 0.0f;
    bool derivative_initialized_ = false;
};

#endif  // JERRO_DRIVERS_PID_CONTROLLER_HPP
