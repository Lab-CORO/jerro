#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <jerro_msgs/msg/motor_speed.hpp>
#include <jerro_msgs/action/auto_tune_pid.hpp>
#include <pigpiod_if2.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>

#include "jerro_drivers/pid_controller.hpp"
#include "jerro_drivers/exponential_filter.hpp"
#include "jerro_drivers/oscillation_detector.hpp"
#include "jerro_drivers/relay_tuner.hpp"
#include "jerro_drivers/velocity_estimator.hpp"

using namespace std::chrono_literals;

class MotorPIDController : public rclcpp::Node
{
public:
    using AutoTunePID = jerro_msgs::action::AutoTunePID;
    using GoalHandleAutoTune = rclcpp_action::ServerGoalHandle<AutoTunePID>;

    MotorPIDController()
    : Node("motor_pid_controller")
    {
        // Initialize hardware
        initPigpio();
        setupMotors();

        // Load parameters
        loadParameters();

        node_start_time_ = std::chrono::steady_clock::now();

        // Create subscribers
        sub_encoder_a_ = this->create_subscription<std_msgs::msg::Int32>(
            "/encoder_a", 10,
            std::bind(&MotorPIDController::encoderACallback, this, std::placeholders::_1));

        sub_encoder_b_ = this->create_subscription<std_msgs::msg::Int32>(
            "/encoder_b", 10,
            std::bind(&MotorPIDController::encoderBCallback, this, std::placeholders::_1));

        sub_motor_speed_ = this->create_subscription<jerro_msgs::msg::MotorSpeed>(
            "/motor/set_speed", 10,
            std::bind(&MotorPIDController::motorSpeedCallback, this, std::placeholders::_1));

        sub_motor_rt_cmd_ = this->create_subscription<jerro_msgs::msg::MotorSpeed>(
            "/motor/motor_RT_cmd", 10,
            std::bind(&MotorPIDController::motorRTCmdCallback, this, std::placeholders::_1));

        // Create control timer (50Hz)
        control_timer_ = this->create_wall_timer(
            20ms, std::bind(&MotorPIDController::controlTimerCallback, this));

        // Create action server
        action_server_ = rclcpp_action::create_server<AutoTunePID>(
            this,
            "/motor/auto_tune",
            std::bind(&MotorPIDController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotorPIDController::handleCancel, this, std::placeholders::_1),
            std::bind(&MotorPIDController::handleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Motor PID controller initialized");
        RCLCPP_INFO(this->get_logger(),
            "  - PID mode: /motor/set_speed (velocity in ticks/sec, range: 0-%.0f)",
            rpmToTicks(max_rpm_));
        RCLCPP_INFO(this->get_logger(), "  - Direct mode: /motor/motor_RT_cmd (PWM values -200 to +200)");
        RCLCPP_INFO(this->get_logger(), "  - Auto-tune action: /motor/auto_tune");
        RCLCPP_INFO(this->get_logger(),
            "  - Encoder resolution: %.1f ticks/rev (quadrature), vitesse max %.0f RPM = %.0f ticks/s",
            ticks_per_revolution_, max_rpm_, rpmToTicks(max_rpm_));
    }

    ~MotorPIDController()
    {
        stopMotors();
        if (pi_ >= 0) {
            pigpio_stop(pi_);
        }
    }

private:
    // Hardware
    int pi_;

    // GPIO pins (PmodDHB1 actual wiring - VERIFIED)
    static constexpr int DIR1 = 5;
    static constexpr int EN1 = 12;  // Hardware PWM canal 0
    static constexpr int DIR2 = 6;
    static constexpr int EN2 = 13;  // Hardware PWM canal 1
    static constexpr int PWM_FREQ = 1000;  // 1kHz - standard and safe frequency

    // H-Bridge enable pins (required for PmodDHB1 to work)
    // Converted from wiringPi numbering to BCM GPIO
    static constexpr int H_BRIDGE_POWER = 26;  // wiringPi 25 -> BCM 26: Alimente H-Drive
    static constexpr int ENABLE_MOTOR1 = 16;   // wiringPi 27 -> BCM 16: Enable Moteur 1
    static constexpr int ENABLE_MOTOR2 = 22;   // wiringPi 3 -> BCM 22: Enable Moteur 2

    // PID Controllers
    PIDController pid_motor_a_;
    PIDController pid_motor_b_;

    // Encoder feedback
    std::atomic<int> encoder_a_count_{0};
    std::atomic<int> encoder_b_count_{0};

    // Sequence et horodatage d'arrivee de chaque message encodeur.
    // /encoder_a et /encoder_b sont publies a 50 Hz. Echantillonner le compteur
    // plus vite que ca renvoie plusieurs fois la MEME valeur, et l'estimateur de
    // vitesse calcule alors des fenetres incoherentes : c'est de l'aliasing, qui
    // se manifeste comme un bruit de mesure enorme (sigma comparable a la
    // moyenne). Les boucles d'identification ne consomment donc qu'un echantillon
    // par message, horodate a son arrivee reelle plutot qu'au moment du sondage.
    std::atomic<uint32_t> encoder_a_seq_{0};
    std::atomic<uint32_t> encoder_b_seq_{0};
    std::atomic<int64_t> encoder_a_stamp_ns_{0};
    std::atomic<int64_t> encoder_b_stamp_ns_{0};
    int last_encoder_a_ = 0;
    int last_encoder_b_ = 0;

    // Target velocities (for PID mode)
    float target_velocity_a_ = 0.0f;
    float target_velocity_b_ = 0.0f;

    // Direct PWM commands (for direct mode)
    float direct_pwm_a_ = 0.0f;
    float direct_pwm_b_ = 0.0f;

    // Control mode
    bool pid_mode_ = true;  // true = PID control, false = direct PWM control

    // Velocity filtering
    ExponentialFilter filter_a_;
    ExponentialFilter filter_b_;

    // Velocity estimation (fenetre adaptative, robuste a basse vitesse)
    VelocityEstimator vel_est_a_;
    VelocityEstimator vel_est_b_;

    // Timing
    std::chrono::steady_clock::time_point last_time_;
    std::chrono::steady_clock::time_point node_start_time_;
    bool first_run_ = true;

    // Auto-tuning: la boucle de controle 50 Hz doit se taire pendant qu'un
    // tuning est en cours, sinon elle ecrit PWM=0 et reset() sur le moteur et le
    // PID en cours d'identification, toutes les 20 ms.
    std::atomic<bool> tuning_active_{false};

    // Auto-tuning parameters
    float Kp_start_ = 0.05f;
    float Kp_increment_ = 0.05f;
    float Kp_max_ = 2.0f;
    float test_duration_per_Kp_ = 8.0f;
    float settling_time_ = 2.0f;
    std::string tune_method_ = "relay";
    ZNVariant zn_variant_ = ZNVariant::NO_OVERSHOOT;
    std::string gains_output_path_;
    float encoder_check_pwm_ = 60.0f;

    // Resolution de l'encodeur. Ne sert pas au calcul de la commande (tout est en
    // ticks/s), mais permet de deriver les bornes de consigne et d'afficher les
    // vitesses en RPM, lisibles par un humain.
    float ticks_per_revolution_ = 1980.0f;
    float max_rpm_ = 30.0f;
    float min_target_rpm_ = 8.0f;

    // Detecteur d'oscillation (chemin 'sweep')
    OscillationDetector detector_template_;

    // Relais (chemin 'relay')
    RelayTuner relay_template_;

    // Garde RAII : garantit que la boucle de controle est rendue au mode normal
    // sur TOUS les chemins de sortie (succes, abort, cancel, exception).
    struct TuningGuard {
        MotorPIDController* self;
        explicit TuningGuard(MotorPIDController* s) : self(s) {
            self->tuning_active_ = true;
        }
        ~TuningGuard() {
            self->haltMotors();
            self->target_velocity_a_ = 0.0f;
            self->target_velocity_b_ = 0.0f;
            self->pid_motor_a_.reset();
            self->pid_motor_b_.reset();
            self->filter_a_.reset();
            self->filter_b_.reset();
            self->vel_est_a_.reset();
            self->vel_est_b_.reset();
            self->first_run_ = true;
            self->tuning_active_ = false;
        }
    };

    // ROS2 components
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_encoder_a_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_encoder_b_;
    rclcpp::Subscription<jerro_msgs::msg::MotorSpeed>::SharedPtr sub_motor_speed_;
    rclcpp::Subscription<jerro_msgs::msg::MotorSpeed>::SharedPtr sub_motor_rt_cmd_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp_action::Server<AutoTunePID>::SharedPtr action_server_;

    void initPigpio()
    {
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon");
            throw std::runtime_error("pigpio initialization failed");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to pigpio daemon");
    }

    void setupMotors()
    {
        // CRITICAL: Configure and enable H-Bridge power pins
        // These pins MUST be HIGH for the H-bridge to work
        RCLCPP_INFO(this->get_logger(), "Configuring H-Bridge enable pins...");
        set_mode(pi_, H_BRIDGE_POWER, PI_OUTPUT);
        set_mode(pi_, ENABLE_MOTOR1, PI_OUTPUT);
        set_mode(pi_, ENABLE_MOTOR2, PI_OUTPUT);

        gpio_write(pi_, H_BRIDGE_POWER, 1);  // Power on H-Bridge
        gpio_write(pi_, ENABLE_MOTOR1, 1);   // Enable Motor 1
        gpio_write(pi_, ENABLE_MOTOR2, 1);   // Enable Motor 2

        RCLCPP_INFO(this->get_logger(), "H-Bridge powered: GPIO %d=HIGH", H_BRIDGE_POWER);
        RCLCPP_INFO(this->get_logger(), "Motor 1 enabled: GPIO %d=HIGH", ENABLE_MOTOR1);
        RCLCPP_INFO(this->get_logger(), "Motor 2 enabled: GPIO %d=HIGH", ENABLE_MOTOR2);

        // Configure motor direction pins as regular outputs
        set_mode(pi_, DIR1, PI_OUTPUT);
        set_mode(pi_, DIR2, PI_OUTPUT);
        gpio_write(pi_, DIR1, 0);
        gpio_write(pi_, DIR2, 0);
        RCLCPP_INFO(this->get_logger(), "Direction pins configured (GPIO %d, %d)", DIR1, DIR2);

        // Configure PWM using hardware_PWM (no need for set_mode - hardware_PWM does it automatically)
        // GPIO 12 = PWM0, GPIO 13 = PWM1 (hardware PWM channels)
        // Duty cycle: 0 to 1000000 (0% to 100%)
        int ret1 = hardware_PWM(pi_, EN1, PWM_FREQ, 0);
        int ret2 = hardware_PWM(pi_, EN2, PWM_FREQ, 0);

        RCLCPP_INFO(this->get_logger(), "Hardware PWM initialized: EN1=%d Hz (ret=%d), EN2=%d Hz (ret=%d)",
                    PWM_FREQ, ret1, PWM_FREQ, ret2);

        RCLCPP_INFO(this->get_logger(), "Hardware PWM ready - EN1: GPIO %d, EN2: GPIO %d @ %d Hz",
                    EN1, EN2, PWM_FREQ);
        RCLCPP_INFO(this->get_logger(), "PmodDHB1 ready with hardware PWM (GPIO 12=PWM0, GPIO 13=PWM1)");
    }

    void loadParameters()
    {
        // Motor A parameters
        this->declare_parameter("motor_a.Kp", 0.15);
        this->declare_parameter("motor_a.Ki", 0.7894);
        this->declare_parameter("motor_a.Kd", 0.00712499);
        this->declare_parameter("motor_a.integral_max", 100.0);
        this->declare_parameter("motor_a.output_max", 200.0);
        this->declare_parameter("motor_a.deadband_pwm", 15.0);
        this->declare_parameter("motor_a.error_deadzone", 5.0);
        this->declare_parameter("motor_a.deadband_blend", 10.0);
        this->declare_parameter("motor_a.derivative_filter_alpha", 0.15);

        float kp_a = this->get_parameter("motor_a.Kp").as_double();
        float ki_a = this->get_parameter("motor_a.Ki").as_double();
        float kd_a = this->get_parameter("motor_a.Kd").as_double();
        pid_motor_a_.setGains(kp_a, ki_a, kd_a);
        pid_motor_a_.integral_max = this->get_parameter("motor_a.integral_max").as_double();
        pid_motor_a_.output_max = this->get_parameter("motor_a.output_max").as_double();
        pid_motor_a_.deadband_pwm = this->get_parameter("motor_a.deadband_pwm").as_double();
        pid_motor_a_.error_deadzone = this->get_parameter("motor_a.error_deadzone").as_double();
        pid_motor_a_.deadband_blend = this->get_parameter("motor_a.deadband_blend").as_double();
        pid_motor_a_.setDerivativeFilterAlpha(
            this->get_parameter("motor_a.derivative_filter_alpha").as_double());

        RCLCPP_INFO(this->get_logger(), "Motor A: Kp=%.3f Ki=%.3f Kd=%.3f",
                    kp_a, ki_a, kd_a);

        // Motor B parameters
        this->declare_parameter("motor_b.Kp", 0.21);
        this->declare_parameter("motor_b.Ki", 2.04878);
        this->declare_parameter("motor_b.Kd", 0.00538125);
        this->declare_parameter("motor_b.integral_max", 100.0);
        this->declare_parameter("motor_b.output_max", 200.0);
        this->declare_parameter("motor_b.deadband_pwm", 15.0);
        this->declare_parameter("motor_b.error_deadzone", 5.0);
        this->declare_parameter("motor_b.deadband_blend", 10.0);
        this->declare_parameter("motor_b.derivative_filter_alpha", 0.15);

        float kp_b = this->get_parameter("motor_b.Kp").as_double();
        float ki_b = this->get_parameter("motor_b.Ki").as_double();
        float kd_b = this->get_parameter("motor_b.Kd").as_double();
        pid_motor_b_.setGains(kp_b, ki_b, kd_b);
        pid_motor_b_.integral_max = this->get_parameter("motor_b.integral_max").as_double();
        pid_motor_b_.output_max = this->get_parameter("motor_b.output_max").as_double();
        pid_motor_b_.deadband_pwm = this->get_parameter("motor_b.deadband_pwm").as_double();
        pid_motor_b_.error_deadzone = this->get_parameter("motor_b.error_deadzone").as_double();
        pid_motor_b_.deadband_blend = this->get_parameter("motor_b.deadband_blend").as_double();
        pid_motor_b_.setDerivativeFilterAlpha(
            this->get_parameter("motor_b.derivative_filter_alpha").as_double());

        RCLCPP_INFO(this->get_logger(), "Motor B: Kp=%.3f Ki=%.3f Kd=%.3f",
                    kp_b, ki_b, kd_b);

        // Control parameters
        this->declare_parameter("control.ticks_per_revolution", 1980.0);
        this->declare_parameter("control.max_rpm", 30.0);
        ticks_per_revolution_ = this->get_parameter("control.ticks_per_revolution").as_double();
        max_rpm_ = this->get_parameter("control.max_rpm").as_double();

        this->declare_parameter("control.velocity_filter_alpha", 0.2);
        this->declare_parameter("control.velocity_min_ticks", 4.0);
        this->declare_parameter("control.velocity_max_window", 0.2);
        float alpha = this->get_parameter("control.velocity_filter_alpha").as_double();
        filter_a_ = ExponentialFilter(alpha);
        filter_b_ = ExponentialFilter(alpha);

        float min_ticks = this->get_parameter("control.velocity_min_ticks").as_double();
        float max_window = this->get_parameter("control.velocity_max_window").as_double();
        vel_est_a_.min_ticks = min_ticks;
        vel_est_a_.max_window = max_window;
        vel_est_b_.min_ticks = min_ticks;
        vel_est_b_.max_window = max_window;

        // Auto-tuning parameters
        this->declare_parameter("auto_tune.Kp_start", 0.05);
        this->declare_parameter("auto_tune.Kp_increment", 0.05);
        this->declare_parameter("auto_tune.Kp_max", 2.0);
        this->declare_parameter("auto_tune.test_duration_per_Kp", 8.0);
        this->declare_parameter("auto_tune.settling_time", 2.0);

        Kp_start_ = this->get_parameter("auto_tune.Kp_start").as_double();
        Kp_increment_ = this->get_parameter("auto_tune.Kp_increment").as_double();
        Kp_max_ = this->get_parameter("auto_tune.Kp_max").as_double();
        test_duration_per_Kp_ = this->get_parameter("auto_tune.test_duration_per_Kp").as_double();
        settling_time_ = this->get_parameter("auto_tune.settling_time").as_double();

        // Methode d'identification et formule de calcul des gains
        this->declare_parameter("auto_tune.method", "relay");
        this->declare_parameter("auto_tune.zn_variant", "no_overshoot");
        this->declare_parameter("auto_tune.output_path",
            "/home/ubuntu/ros2_ws/jerro/jerro_drivers/config/pid_gains.yaml");
        this->declare_parameter("auto_tune.encoder_check_pwm", 200.0);
        this->declare_parameter("auto_tune.min_target_rpm", 8.0);

        tune_method_ = this->get_parameter("auto_tune.method").as_string();
        std::string zn_name = this->get_parameter("auto_tune.zn_variant").as_string();
        zn_variant_ = znVariantFromString(zn_name);
        gains_output_path_ = this->get_parameter("auto_tune.output_path").as_string();
        encoder_check_pwm_ = this->get_parameter("auto_tune.encoder_check_pwm").as_double();
        min_target_rpm_ = this->get_parameter("auto_tune.min_target_rpm").as_double();

        // Parametres du detecteur d'oscillation (chemin 'sweep').
        // Ils existaient dans le YAML mais n'etaient ni declares ni lus :
        // OscillationDetector etait instancie avec ses valeurs codees en dur.
        this->declare_parameter("auto_tune.min_cycles_required", 4);
        this->declare_parameter("auto_tune.period_tolerance", 0.30);
        this->declare_parameter("auto_tune.min_extremum_amplitude", 20.0);
        this->declare_parameter("auto_tune.min_period", 0.2);

        detector_template_.min_cycles_required =
            this->get_parameter("auto_tune.min_cycles_required").as_int();
        detector_template_.period_tolerance =
            this->get_parameter("auto_tune.period_tolerance").as_double();
        detector_template_.min_extremum_amplitude =
            this->get_parameter("auto_tune.min_extremum_amplitude").as_double();
        detector_template_.min_period =
            this->get_parameter("auto_tune.min_period").as_double();

        // Parametres du relais (chemin 'relay')
        this->declare_parameter("auto_tune.relay_amplitude", 40.0);
        this->declare_parameter("auto_tune.relay_hysteresis", 10.0);
        this->declare_parameter("auto_tune.relay_min_cycles", 5);
        this->declare_parameter("auto_tune.relay_skip_cycles", 2);
        this->declare_parameter("auto_tune.relay_bias_rate", 60.0);
        this->declare_parameter("auto_tune.relay_noise_duration", 1.0);
        this->declare_parameter("auto_tune.relay_bias_tolerance", 0.05);
        this->declare_parameter("auto_tune.relay_bias_settle_time", 1.5);
        this->declare_parameter("auto_tune.relay_bias_filter_tc", 0.5);
        this->declare_parameter("auto_tune.relay_hysteresis_max_ratio", 0.25);
        this->declare_parameter("auto_tune.relay_max_switch_gap", 5.0);

        relay_template_.relay_amplitude =
            this->get_parameter("auto_tune.relay_amplitude").as_double();
        relay_template_.min_hysteresis =
            this->get_parameter("auto_tune.relay_hysteresis").as_double();
        relay_template_.min_cycles =
            this->get_parameter("auto_tune.relay_min_cycles").as_int();
        relay_template_.skip_cycles =
            this->get_parameter("auto_tune.relay_skip_cycles").as_int();
        relay_template_.bias_rate =
            this->get_parameter("auto_tune.relay_bias_rate").as_double();
        relay_template_.noise_duration =
            this->get_parameter("auto_tune.relay_noise_duration").as_double();
        relay_template_.bias_tolerance =
            this->get_parameter("auto_tune.relay_bias_tolerance").as_double();
        relay_template_.bias_settle_time =
            this->get_parameter("auto_tune.relay_bias_settle_time").as_double();
        relay_template_.bias_filter_tc =
            this->get_parameter("auto_tune.relay_bias_filter_tc").as_double();
        relay_template_.hysteresis_max_ratio =
            this->get_parameter("auto_tune.relay_hysteresis_max_ratio").as_double();
        relay_template_.max_switch_gap =
            this->get_parameter("auto_tune.relay_max_switch_gap").as_double();
        relay_template_.period_tolerance = detector_template_.period_tolerance;
        relay_template_.bias_max = std::min(
            static_cast<float>(pid_motor_a_.output_max),
            255.0f - relay_template_.relay_amplitude);

        RCLCPP_INFO(this->get_logger(), "Auto-tune: methode=%s, formule=%s",
                    tune_method_.c_str(), znVariantName(zn_variant_));
        RCLCPP_INFO(this->get_logger(), "Auto-tune: gains sauvegardes vers %s",
                    gains_output_path_.c_str());
    }

    float ticksToRpm(float ticks_per_sec) const
    {
        if (ticks_per_revolution_ <= 0.0f) return 0.0f;
        return ticks_per_sec * 60.0f / ticks_per_revolution_;
    }

    float rpmToTicks(float rpm) const
    {
        return rpm * ticks_per_revolution_ / 60.0f;
    }

    int64_t nowNanos() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - node_start_time_).count();
    }

    // Temps monotone depuis le demarrage du noeud, en secondes.
    float nowSeconds() const
    {
        return std::chrono::duration<float>(
            std::chrono::steady_clock::now() - node_start_time_).count();
    }

    void encoderACallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        encoder_a_count_ = msg->data;
        encoder_a_stamp_ns_ = nowNanos();
        encoder_a_seq_.fetch_add(1);  // publie en dernier : count/stamp sont prets
    }

    void encoderBCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        encoder_b_count_ = msg->data;
        encoder_b_stamp_ns_ = nowNanos();
        encoder_b_seq_.fetch_add(1);
    }

    uint32_t encoderSeq(int motor_num) const
    {
        return (motor_num == 1) ? encoder_a_seq_.load() : encoder_b_seq_.load();
    }

    int encoderCount(int motor_num) const
    {
        return (motor_num == 1) ? encoder_a_count_.load() : encoder_b_count_.load();
    }

    float encoderStamp(int motor_num) const
    {
        int64_t ns = (motor_num == 1) ? encoder_a_stamp_ns_.load()
                                      : encoder_b_stamp_ns_.load();
        return static_cast<float>(ns) * 1e-9f;
    }

    void motorSpeedCallback(const jerro_msgs::msg::MotorSpeed::SharedPtr msg)
    {
        // Switch to PID mode
        if (!pid_mode_) {
            pid_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Switched to PID control mode");
        }

        // Setpoint is in ticks/sec (encoder ticks per second)
        // La borne haute est derivee de control.ticks_per_revolution et
        // control.max_rpm : elle suit automatiquement un changement de moteur.
        target_velocity_a_ = msg->motor_speed_a;
        target_velocity_b_ = msg->motor_speed_b;

        // Warn if setpoint seems unusually high
        float max_ticks = rpmToTicks(max_rpm_);
        if (std::abs(target_velocity_a_) > max_ticks || std::abs(target_velocity_b_) > max_ticks) {
            RCLCPP_WARN(this->get_logger(),
                "High velocity setpoint: A=%.1f B=%.1f ticks/sec (%.1f/%.1f RPM, max %.0f ticks/s = %.0f RPM)",
                target_velocity_a_, target_velocity_b_,
                ticksToRpm(target_velocity_a_), ticksToRpm(target_velocity_b_),
                max_ticks, max_rpm_);
        }

        RCLCPP_DEBUG(this->get_logger(), "PID setpoint: A=%.1f B=%.1f ticks/sec",
                     target_velocity_a_, target_velocity_b_);
    }

    void motorRTCmdCallback(const jerro_msgs::msg::MotorSpeed::SharedPtr msg)
    {
        // Switch to direct PWM mode
        if (pid_mode_) {
            pid_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Switched to direct PWM control mode");
        }

        // Store direct PWM commands (values are interpreted as PWM: -200 to +200)
        direct_pwm_a_ = msg->motor_speed_a;
        direct_pwm_b_ = msg->motor_speed_b;

        RCLCPP_DEBUG(this->get_logger(), "Direct PWM cmd: A=%.1f B=%.1f",
                     direct_pwm_a_, direct_pwm_b_);
    }

    void controlTimerCallback()
    {
        // Pendant un auto-tune, le thread de tuning est seul proprietaire des
        // moteurs et des objets PID. Sans ce garde, ce callback ecrirait
        // setMotorPWM(x, 0) et pid.reset() toutes les 20 ms par-dessus la
        // commande du tuning (target_velocity_ vaut 0 pendant l'identification),
        // hachant le signal et corrompant l'etat du PID depuis un autre thread.
        if (tuning_active_.load()) {
            return;
        }

        auto current_time = std::chrono::steady_clock::now();

        if (first_run_) {
            last_time_ = current_time;
            first_run_ = false;
            return;
        }

        // Calculate dt
        float dt = std::chrono::duration<float>(current_time - last_time_).count();
        last_time_ = current_time;

        // Sanity check on dt
        if (dt < 0.001f || dt > 1.0f) {
            RCLCPP_WARN(this->get_logger(), "Invalid dt: %.3f", dt);
            return;
        }

        // Measure velocities (always, for both modes)
        int current_encoder_a = encoder_a_count_.load();
        int current_encoder_b = encoder_b_count_.load();

        last_encoder_a_ = current_encoder_a;
        last_encoder_b_ = current_encoder_b;

        // Estimation a fenetre adaptative : a vitesse elevee la fenetre reste
        // courte, a basse vitesse elle s'allonge pour recuperer de la resolution
        // au lieu de sauter entre 0 et 1/dt ticks/s.
        float t_now = nowSeconds();
        float velocity_raw_a = vel_est_a_.update(current_encoder_a, t_now);
        float velocity_raw_b = vel_est_b_.update(current_encoder_b, t_now);

        // Filter velocities
        float velocity_a = filter_a_.update(velocity_raw_a);
        float velocity_b = filter_b_.update(velocity_raw_b);

        // Direct PWM mode - apply commands directly without PID
        if (!pid_mode_) {
            setMotorPWM(1, direct_pwm_a_);
            setMotorPWM(2, direct_pwm_b_);
            return;
        }

        // PID mode - compute PID output based on measured velocity
        // Compute PID outputs for Motor A
        float pwm_a;
        if (std::abs(target_velocity_a_) < 0.1f) {
            // Target is zero - stop motor and reset PID
            pid_motor_a_.reset();
            pwm_a = 0.0f;
        } else {
            pwm_a = pid_motor_a_.compute(target_velocity_a_, velocity_a, dt);
        }

        // Compute PID outputs for Motor B
        float pwm_b;
        if (std::abs(target_velocity_b_) < 0.1f) {
            // Target is zero - stop motor and reset PID
            pid_motor_b_.reset();
            pwm_b = 0.0f;
        } else {
            pwm_b = pid_motor_b_.compute(target_velocity_b_, velocity_b, dt);
        }

        // Apply to motors
        setMotorPWM(1, pwm_a);
        setMotorPWM(2, pwm_b);
    }

    void setMotorPWM(int motor_num, float pwm_value)
    {
        if (motor_num == 1) {
            // Motor A (Motor 1)
            int ret;
            if (pwm_value >= 0) {
                gpio_write(pi_, DIR1, 1);
                int duty_255 = static_cast<int>(std::abs(pwm_value));
                if (duty_255 > 255) duty_255 = 255;
                // Convert 0-255 to 0-1000000 for hardware_PWM
                int duty_1M = (duty_255 * 1000000) / 255;
                ret = hardware_PWM(pi_, EN1, PWM_FREQ, duty_1M);
                RCLCPP_DEBUG(this->get_logger(), "Motor A: PWM=%.1f, DIR=1, duty=%d/1M, ret=%d", pwm_value, duty_1M, ret);
            } else {
                gpio_write(pi_, DIR1, 0);
                int duty_255 = static_cast<int>(std::abs(pwm_value));
                if (duty_255 > 255) duty_255 = 255;
                // Convert 0-255 to 0-1000000 for hardware_PWM
                int duty_1M = (duty_255 * 1000000) / 255;
                ret = hardware_PWM(pi_, EN1, PWM_FREQ, duty_1M);
                RCLCPP_DEBUG(this->get_logger(), "Motor A: PWM=%.1f, DIR=0, duty=%d/1M, ret=%d", pwm_value, duty_1M, ret);
            }
            if (ret < 0) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "hardware_PWM failed for Motor A (ret=%d)", ret);
            }
        } else if (motor_num == 2) {
            // Motor B (Motor 2)
            int ret;
            if (pwm_value >= 0) {
                gpio_write(pi_, DIR2, 1);
                int duty_255 = static_cast<int>(std::abs(pwm_value));
                if (duty_255 > 255) duty_255 = 255;
                // Convert 0-255 to 0-1000000 for hardware_PWM
                int duty_1M = (duty_255 * 1000000) / 255;
                ret = hardware_PWM(pi_, EN2, PWM_FREQ, duty_1M);
                RCLCPP_DEBUG(this->get_logger(), "Motor B: PWM=%.1f, DIR=1, duty=%d/1M, ret=%d", pwm_value, duty_1M, ret);
            } else {
                gpio_write(pi_, DIR2, 0);
                int duty_255 = static_cast<int>(std::abs(pwm_value));
                if (duty_255 > 255) duty_255 = 255;
                // Convert 0-255 to 0-1000000 for hardware_PWM
                int duty_1M = (duty_255 * 1000000) / 255;
                ret = hardware_PWM(pi_, EN2, PWM_FREQ, duty_1M);
                RCLCPP_DEBUG(this->get_logger(), "Motor B: PWM=%.1f, DIR=0, duty=%d/1M, ret=%d", pwm_value, duty_1M, ret);
            }
            if (ret < 0) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "hardware_PWM failed for Motor B (ret=%d)", ret);
            }
        }
    }

    // Arrete le mouvement sans couper le pont en H.
    // A utiliser partout dans l'auto-tune : stopMotors() coupe l'alimentation du
    // pont et rien ne la retablit (setupMotors() n'est appele que dans le
    // constructeur), ce qui rendait les moteurs inutilisables jusqu'au
    // redemarrage du noeud apres un echec de tuning.
    void haltMotors()
    {
        setMotorPWM(1, 0.0f);
        setMotorPWM(2, 0.0f);
    }

    void stopMotors()
    {
        // Stop Hardware PWM (duty=0)
        hardware_PWM(pi_, EN1, 0, 0);
        hardware_PWM(pi_, EN2, 0, 0);

        // Disable H-Bridge (important for safety and power saving)
        gpio_write(pi_, H_BRIDGE_POWER, 0);
        gpio_write(pi_, ENABLE_MOTOR1, 0);
        gpio_write(pi_, ENABLE_MOTOR2, 0);

        RCLCPP_INFO(this->get_logger(), "Motors stopped and H-Bridge disabled");
    }

    // Action server handlers
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const AutoTunePID::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received auto-tune request for motor(s) %d", goal->motor_select);

        // Deux tunings simultanes se battraient pour les memes moteurs et les
        // memes objets PID.
        if (tuning_active_.load()) {
            RCLCPP_WARN(this->get_logger(), "Auto-tune deja en cours, goal rejete");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->motor_select != AutoTunePID::Goal::MOTOR_A &&
            goal->motor_select != AutoTunePID::Goal::MOTOR_B &&
            goal->motor_select != AutoTunePID::Goal::BOTH_MOTORS) {
            RCLCPP_WARN(this->get_logger(), "motor_select invalide: %d (attendu 1, 2 ou 3)",
                        goal->motor_select);
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleAutoTune> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel auto-tune");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleAutoTune> goal_handle)
    {
        // Execute in a separate thread to avoid blocking the executor
        std::thread{std::bind(&MotorPIDController::executeAutoTune, this, std::placeholders::_1), goal_handle}.detach();
    }

    void executeAutoTune(const std::shared_ptr<GoalHandleAutoTune> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<AutoTunePID::Result>();
        auto feedback = std::make_shared<AutoTunePID::Feedback>();

        RCLCPP_INFO(this->get_logger(), "Starting auto-tune...");

        // Initialize result values
        result->success = false;
        result->ku_a = 0;
        result->tu_a = 0;
        result->kp_a = 0;
        result->ki_a = 0;
        result->kd_a = 0;
        result->ku_b = 0;
        result->tu_b = 0;
        result->kp_b = 0;
        result->ki_b = 0;
        result->kd_b = 0;

        // Prend possession des moteurs et fait taire la boucle 50 Hz jusqu'a la
        // sortie de cette fonction, quel que soit le chemin emprunte.
        TuningGuard guard(this);

        // Auto-tune based on motor selection
        bool tune_a = (goal->motor_select == AutoTunePID::Goal::MOTOR_A ||
                       goal->motor_select == AutoTunePID::Goal::BOTH_MOTORS);
        bool tune_b = (goal->motor_select == AutoTunePID::Goal::MOTOR_B ||
                       goal->motor_select == AutoTunePID::Goal::BOTH_MOTORS);

        float target_velocity = goal->target_velocity;
        if (target_velocity <= 0.0f) {
            result->message = "target_velocity doit etre > 0 (recu " +
                              std::to_string(target_velocity) + ")";
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
            return;
        }

        // Garde-fou de point de fonctionnement. Une consigne exprimee en ticks/s
        // ne dit rien sur le regime physique vise : avec 1980 ticks/tour,
        // 200 ticks/s valent 6.1 RPM, c'est-a-dire un rampement en plein
        // frottement sec - le pire point pour identifier une dynamique.
        float target_rpm = ticksToRpm(target_velocity);
        RCLCPP_INFO(this->get_logger(),
                    "Consigne d'identification: %.0f ticks/s = %.1f RPM (%.1f ticks/tour)",
                    target_velocity, target_rpm, ticks_per_revolution_);

        if (target_rpm < min_target_rpm_) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "consigne trop basse: %.0f ticks/s = %.1f RPM, minimum %.1f RPM "
                "(soit %.0f ticks/s). Identifier un moteur au ralenti donne un "
                "resultat domine par le frottement sec, pas par sa dynamique",
                target_velocity, target_rpm, min_target_rpm_, rpmToTicks(min_target_rpm_));
            result->message = buf;
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
            return;
        }

        if (target_rpm > max_rpm_) {
            RCLCPP_WARN(this->get_logger(),
                "Consigne au-dela de la vitesse a vide (%.1f RPM > %.1f RPM): "
                "le moteur risque de saturer avant d'osciller",
                target_rpm, max_rpm_);
        }

        float max_duration = goal->max_duration > 0.0f ? goal->max_duration : 120.0f;

        float Ku_a = 0, Tu_a = 0;
        float Ku_b = 0, Tu_b = 0;
        std::string failure;

        // Tune Motor A
        if (tune_a) {
            feedback->status = "Auto-tuning Motor A...";
            feedback->current_motor = AutoTunePID::Goal::MOTOR_A;
            feedback->progress = 0.0;
            goal_handle->publish_feedback(feedback);

            encoder_a_count_ = 0;
            last_encoder_a_ = 0;
            if (tuneMotor(1, target_velocity, max_duration, goal_handle, Ku_a, Tu_a, failure)) {
                result->ku_a = Ku_a;
                result->tu_a = Tu_a;
                result->kp_a = pid_motor_a_.Kp;
                result->ki_a = pid_motor_a_.Ki;
                result->kd_a = pid_motor_a_.Kd;
                RCLCPP_INFO(this->get_logger(), "Motor A tuned: Ku=%.3f Tu=%.3f", Ku_a, Tu_a);
            } else {
                result->message = "Moteur A: " + failure;
                RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
                finishFailed(goal_handle, result);
                return;
            }
        }

        // Tune Motor B
        if (tune_b) {
            feedback->status = "Auto-tuning Motor B...";
            feedback->current_motor = AutoTunePID::Goal::MOTOR_B;
            feedback->progress = tune_a ? 0.5 : 0.0;
            goal_handle->publish_feedback(feedback);

            encoder_b_count_ = 0;
            last_encoder_b_ = 0;
            if (tuneMotor(2, target_velocity, max_duration, goal_handle, Ku_b, Tu_b, failure)) {
                result->ku_b = Ku_b;
                result->tu_b = Tu_b;
                result->kp_b = pid_motor_b_.Kp;
                result->ki_b = pid_motor_b_.Ki;
                result->kd_b = pid_motor_b_.Kd;
                RCLCPP_INFO(this->get_logger(), "Motor B tuned: Ku=%.3f Tu=%.3f", Ku_b, Tu_b);
            } else {
                result->message = "Moteur B: " + failure;
                RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
                finishFailed(goal_handle, result);
                return;
            }
        }

        // Sauvegarde. On ecrit les gains VIVANTS des deux PID, pas les champs du
        // resultat : quand un seul moteur est tune, les champs de l'autre valent
        // 0 et ecrasaient ses gains dans le fichier.
        std::string save_error;
        bool saved = saveGainsToYAML(gains_output_path_, save_error);

        result->success = true;
        if (saved) {
            result->message = "Auto-tuning termine, gains ecrits dans " + gains_output_path_ +
                              " (colcon build requis pour les propager dans share/)";
        } else {
            // On ne pretend pas avoir sauvegarde : l'ancien code loggait l'echec
            // et retournait quand meme success=true.
            result->success = false;
            result->message = "Auto-tuning reussi mais sauvegarde impossible: " + save_error;
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            finishFailed(goal_handle, result);
            return;
        }

        feedback->status = "Auto-tuning completed";
        feedback->progress = 1.0;
        goal_handle->publish_feedback(feedback);
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Auto-tuning completed successfully");
    }

    // Termine le goal en distinguant annulation et echec, pour que l'appelant
    // voie CANCELED plutot qu'ABORTED quand il a lui-meme annule.
    void finishFailed(const std::shared_ptr<GoalHandleAutoTune>& goal_handle,
                      std::shared_ptr<AutoTunePID::Result> result)
    {
        result->success = false;
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->abort(result);
        }
    }

    // Aiguillage entre les deux methodes d'identification.
    bool tuneMotor(int motor_num, float target_velocity, float max_duration,
                   std::shared_ptr<GoalHandleAutoTune> goal_handle,
                   float& Ku_out, float& Tu_out, std::string& failure)
    {
        // Verification materielle prealable : distingue en quelques secondes une
        // panne (moteur, connecteur, encodeur) d'un probleme de reglage, au lieu
        // d'attendre l'expiration du budget de temps.
        if (!checkEncoderFeedback(motor_num, failure)) {
            return false;
        }

        if (tune_method_ == "sweep") {
            return autoTuneMotor(motor_num, target_velocity, max_duration,
                                 goal_handle, Ku_out, Tu_out, failure);
        }
        return autoTuneRelay(motor_num, target_velocity, max_duration,
                             goal_handle, Ku_out, Tu_out, failure);
    }

    // Applique un PWM connu et verifie que le compteur encodeur progresse.
    bool checkEncoderFeedback(int motor_num, std::string& failure)
    {
        using namespace std::chrono;

        auto readCount = [this](int m) {
            return (m == 1) ? encoder_a_count_.load() : encoder_b_count_.load();
        };

        int start_count = readCount(motor_num);
        setMotorPWM(motor_num, encoder_check_pwm_);
        std::this_thread::sleep_for(1s);
        int end_count = readCount(motor_num);
        setMotorPWM(motor_num, 0.0f);
        std::this_thread::sleep_for(300ms);

        int delta = std::abs(end_count - start_count);
        RCLCPP_INFO(this->get_logger(),
                    "Verification encodeur moteur %d: %d ticks a PWM=%.0f",
                    motor_num, delta, encoder_check_pwm_);

        if (delta < 10) {
            failure = "aucun retour encodeur (" + std::to_string(delta) +
                      " ticks a PWM=" + std::to_string(static_cast<int>(encoder_check_pwm_)) +
                      ") - verifier le cablage moteur/encodeur et l'alimentation du pont en H";
            return false;
        }
        return true;
    }

    // Identification par retour a relais (Astrom-Hagglund).
    bool autoTuneRelay(int motor_num, float target_velocity, float max_duration,
                       std::shared_ptr<GoalHandleAutoTune> goal_handle,
                       float& Ku_out, float& Tu_out, std::string& failure)
    {
        using namespace std::chrono;

        PIDController* pid = (motor_num == 1) ? &pid_motor_a_ : &pid_motor_b_;
        VelocityEstimator estimator;
        estimator.min_ticks = vel_est_a_.min_ticks;
        estimator.max_window = vel_est_a_.max_window;

        RelayTuner tuner = relay_template_;
        tuner.start(target_velocity);

        RCLCPP_INFO(this->get_logger(),
                    "Relais moteur %d: consigne=%.1f ticks/s, d=%.0f PWM",
                    motor_num, target_velocity, tuner.relay_amplitude);

        auto start_time = steady_clock::now();
        auto last_feedback = start_time;
        auto last_state = tuner.state();

        uint32_t last_seq = encoderSeq(motor_num);
        float last_sample_t = -1.0f;
        float velocity = 0.0f;

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                failure = "annule par l'utilisateur";
                haltMotors();
                return false;
            }

            auto current_time = steady_clock::now();
            float elapsed = duration<float>(current_time - start_time).count();
            if (elapsed > max_duration) {
                tuner.timeout();
                failure = tuner.message;
                haltMotors();
                return false;
            }

            // Un pas par message encodeur : sonder plus vite ne ferait que
            // relire le meme compteur et fabriquer du bruit (aliasing).
            uint32_t seq = encoderSeq(motor_num);
            if (seq == last_seq) {
                std::this_thread::sleep_for(2ms);
                continue;
            }
            last_seq = seq;

            int count = encoderCount(motor_num);
            float sample_t = encoderStamp(motor_num);

            if (last_sample_t < 0.0f) {
                last_sample_t = sample_t;
                estimator.update(count, sample_t);
                continue;
            }

            float dt = sample_t - last_sample_t;
            last_sample_t = sample_t;
            if (dt <= 0.0f || dt > 0.5f) {
                continue;  // horodatage aberrant (message perdu, reprise)
            }

            velocity = estimator.update(count, sample_t);

            float pwm = tuner.update(velocity, dt);

            if (tuner.state() == RelayTuner::State::DONE) {
                haltMotors();
                Ku_out = tuner.Ku;
                Tu_out = tuner.Tu;
                RCLCPP_INFO(this->get_logger(),
                            "  Cycle limite: a=%.1f ticks/s, h=%.1f, bias=%.0f PWM",
                            tuner.amplitude, tuner.hysteresis, tuner.bias);
                RCLCPP_INFO(this->get_logger(), "  Ku = %.3f, Tu = %.3f s", Ku_out, Tu_out);
                if (!tuner.note.empty()) {
                    RCLCPP_WARN(this->get_logger(), "  %s", tuner.note.c_str());
                }
                pid->calculateZieglerNicholsGains(Ku_out, Tu_out, zn_variant_);
                RCLCPP_INFO(this->get_logger(),
                            "Motor %d tuned (%s): Kp=%.4f Ki=%.4f Kd=%.5f",
                            motor_num, znVariantName(zn_variant_),
                            pid->Kp, pid->Ki, pid->Kd);
                return true;
            }

            if (tuner.state() == RelayTuner::State::FAILED) {
                failure = tuner.message;
                haltMotors();
                return false;
            }

            setMotorPWM(motor_num, pwm);

            // Feedback a ~2 Hz, et immediatement a chaque changement de phase
            if (tuner.state() != last_state ||
                duration<float>(current_time - last_feedback).count() > 0.5f) {
                last_state = tuner.state();
                last_feedback = current_time;

                auto feedback = std::make_shared<AutoTunePID::Feedback>();
                feedback->status = std::string("Relais [") + tuner.stateName() +
                                   "] v=" + std::to_string(static_cast<int>(velocity)) +
                                   " ticks/s, cycles=" + std::to_string(tuner.cyclesCollected());
                feedback->current_kp = pid->Kp;
                feedback->current_motor = motor_num;
                feedback->progress = std::min(0.95f, elapsed / max_duration);
                goal_handle->publish_feedback(feedback);
            }

        }

        failure = "arret du noeud pendant l'identification";
        haltMotors();
        return false;
    }

    bool autoTuneMotor(int motor_num, float target_velocity, float max_duration,
                       std::shared_ptr<GoalHandleAutoTune> goal_handle,
                       float& Ku_out, float& Tu_out, std::string& failure)
    {
        using namespace std::chrono;

        // Get PID controller reference
        PIDController* pid = (motor_num == 1) ? &pid_motor_a_ : &pid_motor_b_;

        // Save original gains
        float original_Kp = pid->Kp;
        float original_Ki = pid->Ki;
        float original_Kd = pid->Kd;

        // Le budget de temps doit couvrir tout le balayage, sinon on echoue par
        // expiration sans jamais atteindre Kp_max. On previent explicitement au
        // lieu de laisser l'utilisateur decouvrir le probleme 100 s plus tard.
        float steps = (Kp_max_ - Kp_start_) / Kp_increment_;
        float needed = steps * test_duration_per_Kp_;
        if (needed > max_duration) {
            float reachable = Kp_start_ + Kp_increment_ * (max_duration / test_duration_per_Kp_);
            RCLCPP_WARN(this->get_logger(),
                "Budget insuffisant: le balayage Kp=%.2f..%.2f par pas de %.3f a %.1f s/palier "
                "demande ~%.0f s, mais max_duration=%.0f s. Kp maximum atteignable: %.2f",
                Kp_start_, Kp_max_, Kp_increment_, test_duration_per_Kp_,
                needed, max_duration, reachable);
        }

        // Initialize for pure proportional control
        pid->Ki = 0.0f;
        pid->Kd = 0.0f;
        pid->Kp = Kp_start_;
        pid->reset();

        RCLCPP_INFO(this->get_logger(), "Auto-tuning motor %d: target=%.1f ticks/sec",
                    motor_num, target_velocity);

        auto start_time = steady_clock::now();
        float best_amplitude = 0.0f;

        // Sweep Kp
        while (pid->Kp <= Kp_max_) {
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                failure = "annule par l'utilisateur";
                pid->setGains(original_Kp, original_Ki, original_Kd);
                haltMotors();
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "Testing Kp = %.3f", pid->Kp);

            // Publish feedback
            auto feedback = std::make_shared<AutoTunePID::Feedback>();
            feedback->status = "Testing Kp = " + std::to_string(pid->Kp);
            feedback->current_kp = pid->Kp;
            feedback->current_motor = motor_num;
            feedback->progress = (pid->Kp - Kp_start_) / (Kp_max_ - Kp_start_);
            goal_handle->publish_feedback(feedback);

            OscillationDetector detector = detector_template_;
            pid->reset();

            VelocityEstimator estimator;
            estimator.min_ticks = vel_est_a_.min_ticks;
            estimator.max_window = vel_est_a_.max_window;

            auto test_start = steady_clock::now();
            uint32_t last_seq = encoderSeq(motor_num);
            float last_sample_t = -1.0f;

            bool oscillation_found = false;
            float Tu_detected = 0;
            float amp_min = 0.0f, amp_max = 0.0f;
            bool amp_init = false;

            // Control loop for this Kp value
            while (rclcpp::ok()) {
                auto current_time = steady_clock::now();
                float elapsed = duration<float>(current_time - test_start).count();

                // Check timeout
                if (elapsed > test_duration_per_Kp_) {
                    RCLCPP_INFO(this->get_logger(),
                                "  No oscillation at Kp=%.3f (amplitude erreur observee: %.1f ticks/s)",
                                pid->Kp, amp_max - amp_min);
                    break;
                }

                // Le budget global doit aussi etre verifie ICI : sinon on ne le
                // teste qu'apres un palier complet et on depasse largement.
                if (duration<float>(current_time - start_time).count() > max_duration) {
                    break;
                }

                // Un pas par message encodeur (cf. aliasing, meme raison que
                // dans autoTuneRelay).
                uint32_t seq = encoderSeq(motor_num);
                if (seq == last_seq) {
                    std::this_thread::sleep_for(2ms);
                    continue;
                }
                last_seq = seq;

                int current_count = encoderCount(motor_num);
                float sample_t = encoderStamp(motor_num);

                if (last_sample_t < 0.0f) {
                    last_sample_t = sample_t;
                    estimator.update(current_count, sample_t);
                    continue;
                }

                float dt = sample_t - last_sample_t;
                last_sample_t = sample_t;
                if (dt <= 0.0f || dt > 0.5f) {
                    continue;
                }

                float velocity = estimator.update(current_count, sample_t);

                // Compute PID
                float pwm = pid->compute(target_velocity, velocity, dt);
                setMotorPWM(motor_num, pwm);

                // Detect oscillation (after settling)
                if (elapsed > settling_time_) {
                    if (!amp_init) {
                        amp_min = amp_max = pid->raw_error;
                        amp_init = true;
                    }
                    amp_min = std::min(amp_min, pid->raw_error);
                    amp_max = std::max(amp_max, pid->raw_error);

                    if (detector.detectOscillation(pid->raw_error, dt, Tu_detected)) {
                        oscillation_found = true;
                        Ku_out = pid->Kp;
                        Tu_out = Tu_detected;

                        RCLCPP_INFO(this->get_logger(), "  OSCILLATION DETECTED!");
                        RCLCPP_INFO(this->get_logger(), "  Ku = %.3f, Tu = %.3f sec", Ku_out, Tu_out);
                        break;
                    }
                }

            }

            best_amplitude = std::max(best_amplitude, amp_max - amp_min);

            // Stop motor
            setMotorPWM(motor_num, 0);
            std::this_thread::sleep_for(500ms);

            if (oscillation_found) {
                // Success! Calculate gains from Ku/Tu
                pid->calculateZieglerNicholsGains(Ku_out, Tu_out, zn_variant_);
                RCLCPP_INFO(this->get_logger(),
                            "Motor %d tuned (%s): Kp=%.4f Ki=%.4f Kd=%.5f",
                            motor_num, znVariantName(zn_variant_),
                            pid->Kp, pid->Ki, pid->Kd);
                return true;
            }

            // Increment Kp
            pid->Kp += Kp_increment_;

            // Check global timeout
            float total_elapsed = duration<float>(steady_clock::now() - start_time).count();
            if (total_elapsed > max_duration) {
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "timeout: budget de %.0f s epuise, atteint Kp=%.2f sur Kp_max=%.2f "
                    "(amplitude d'erreur max observee: %.1f ticks/s). "
                    "Augmenter max_duration a ~%.0f s ou utiliser auto_tune.method=relay",
                    max_duration, pid->Kp, Kp_max_, best_amplitude, needed);
                failure = buf;
                pid->setGains(original_Kp, original_Ki, original_Kd);
                haltMotors();
                return false;
            }
        }

        // Failed to find oscillation
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Kp_max=%.2f atteint sans oscillation soutenue "
            "(amplitude d'erreur max observee: %.1f ticks/s, seuil de detection: %.1f). "
            "Augmenter Kp_max, ou baisser auto_tune.min_extremum_amplitude",
            Kp_max_, best_amplitude, detector_template_.min_extremum_amplitude);
        failure = buf;
        pid->setGains(original_Kp, original_Ki, original_Kd);
        haltMotors();
        return false;
    }

    // Ecrit les gains actuellement actifs des DEUX moteurs. Utiliser l'etat vivant
    // des PID (et non les champs du resultat) evite d'ecraser par des zeros les
    // gains du moteur qui n'a pas ete tune lors de cet appel.
    bool saveGainsToYAML(const std::string& filepath, std::string& error)
    {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            error = "ouverture impossible de " + filepath;
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", filepath.c_str());
            return false;
        }

        file << std::fixed << std::setprecision(6);
        file << "# Auto-tuned PID gains\n";
        file << "# Generated by motor_pid_controller auto-tune action\n";
        file << "# methode: " << tune_method_ << ", formule: " << znVariantName(zn_variant_) << "\n\n";
        file << "motor_pid_controller:\n";
        file << "  ros__parameters:\n";
        file << "    motor_a:\n";
        file << "      Kp: " << pid_motor_a_.Kp << "\n";
        file << "      Ki: " << pid_motor_a_.Ki << "\n";
        file << "      Kd: " << pid_motor_a_.Kd << "\n";
        file << "    motor_b:\n";
        file << "      Kp: " << pid_motor_b_.Kp << "\n";
        file << "      Ki: " << pid_motor_b_.Ki << "\n";
        file << "      Kd: " << pid_motor_b_.Kd << "\n";

        file.close();
        if (file.fail()) {
            error = "erreur d'ecriture dans " + filepath;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Saved tuned gains to %s", filepath.c_str());
        return true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<MotorPIDController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_pid_controller"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
