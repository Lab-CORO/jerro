#ifndef JERRO_DRIVERS_PID_CONTROLLER_HPP
#define JERRO_DRIVERS_PID_CONTROLLER_HPP

#include <cmath>
#include <iostream>

class PIDController {
public:
    // Gains
    float Kp = 0.1f;
    float Ki = 0.0f;
    float Kd = 0.0f;

    // State
    float error = 0;
    float raw_error = 0;       // Erreur brute avant deadzone (pour détection oscillation)
    float last_error = 0;
    float integral = 0;
    float derivative = 0;

    // Limits
    float integral_max = 100.0f;
    float output_max = 200.0f;
    float deadband_pwm = 15.0f;
    float error_deadzone = 5.0f;

    PIDController() = default;

    float compute(float setpoint, float measured, float dt) {
        raw_error = setpoint - measured;  // Stocker erreur brute AVANT deadzone
        error = raw_error;                // Copie pour traitement

        if (std::abs(error) < error_deadzone) {
            error = 0;  // Deadzone appliquée seulement à 'error', pas 'raw_error'
        }

        integral += error * dt;
        if (integral > integral_max) integral = integral_max;
        if (integral < -integral_max) integral = -integral_max;

        derivative = (error - last_error) / dt;

        float output = Kp * error + Ki * integral + Kd * derivative;

        if (output > output_max) output = output_max;
        if (output < -output_max) output = -output_max;

        // Compensation zone morte
        if (output > 0) {
            output = output + deadband_pwm;
            if (output > 255) output = 255;
        } else if (output < 0) {
            output = output - deadband_pwm;
            if (output < -255) output = -255;
        }

        last_error = error;
        return output;
    }

    void reset() {
        error = 0;
        last_error = 0;
        integral = 0;
        derivative = 0;
    }

    void setDeadband(float pwm) {
        deadband_pwm = pwm;
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void calculateZieglerNicholsGains(float Ku, float Tu) {
        // Formules Ziegler-Nichols classiques pour PID
        float new_Kp = 0.6f * Ku;
        float new_Ki = 1.2f * Ku / Tu;
        float new_Kd = 0.075f * Ku * Tu;

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
};

#endif  // JERRO_DRIVERS_PID_CONTROLLER_HPP
