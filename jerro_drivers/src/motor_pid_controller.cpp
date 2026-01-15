#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <jerro_msgs/msg/motor_speed.hpp>
#include <jerro_msgs/action/auto_tune_pid.hpp>
#include <pigpiod_if2.h>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>

#include "jerro_drivers/pid_controller.hpp"
#include "jerro_drivers/exponential_filter.hpp"
#include "jerro_drivers/oscillation_detector.hpp"

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
        RCLCPP_INFO(this->get_logger(), "  - PID mode: /motor/set_speed (velocity in ticks/sec, range: 0-2500)");
        RCLCPP_INFO(this->get_logger(), "  - Direct mode: /motor/motor_RT_cmd (PWM values -200 to +200)");
        RCLCPP_INFO(this->get_logger(), "  - Auto-tune action: /motor/auto_tune");
        RCLCPP_INFO(this->get_logger(), "  - Encoder resolution: 158.2 PPR (632.8 ticks/rev in quadrature)");
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

    // Timing
    std::chrono::steady_clock::time_point last_time_;
    bool first_run_ = true;

    // Auto-tuning parameters
    float Kp_start_ = 0.05f;
    float Kp_increment_ = 0.05f;
    float Kp_max_ = 2.0f;
    float test_duration_per_Kp_ = 8.0f;
    float settling_time_ = 2.0f;

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

        float kp_a = this->get_parameter("motor_a.Kp").as_double();
        float ki_a = this->get_parameter("motor_a.Ki").as_double();
        float kd_a = this->get_parameter("motor_a.Kd").as_double();
        pid_motor_a_.setGains(kp_a, ki_a, kd_a);
        pid_motor_a_.integral_max = this->get_parameter("motor_a.integral_max").as_double();
        pid_motor_a_.output_max = this->get_parameter("motor_a.output_max").as_double();
        pid_motor_a_.deadband_pwm = this->get_parameter("motor_a.deadband_pwm").as_double();
        pid_motor_a_.error_deadzone = this->get_parameter("motor_a.error_deadzone").as_double();

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

        float kp_b = this->get_parameter("motor_b.Kp").as_double();
        float ki_b = this->get_parameter("motor_b.Ki").as_double();
        float kd_b = this->get_parameter("motor_b.Kd").as_double();
        pid_motor_b_.setGains(kp_b, ki_b, kd_b);
        pid_motor_b_.integral_max = this->get_parameter("motor_b.integral_max").as_double();
        pid_motor_b_.output_max = this->get_parameter("motor_b.output_max").as_double();
        pid_motor_b_.deadband_pwm = this->get_parameter("motor_b.deadband_pwm").as_double();
        pid_motor_b_.error_deadzone = this->get_parameter("motor_b.error_deadzone").as_double();

        RCLCPP_INFO(this->get_logger(), "Motor B: Kp=%.3f Ki=%.3f Kd=%.3f",
                    kp_b, ki_b, kd_b);

        // Control parameters
        this->declare_parameter("control.velocity_filter_alpha", 0.2);
        float alpha = this->get_parameter("control.velocity_filter_alpha").as_double();
        filter_a_ = ExponentialFilter(alpha);
        filter_b_ = ExponentialFilter(alpha);

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
    }

    void encoderACallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        encoder_a_count_ = msg->data;
    }

    void encoderBCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        encoder_b_count_ = msg->data;
    }

    void motorSpeedCallback(const jerro_msgs::msg::MotorSpeed::SharedPtr msg)
    {
        // Switch to PID mode
        if (!pid_mode_) {
            pid_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Switched to PID control mode");
        }

        // Setpoint is in ticks/sec (encoder ticks per second)
        // Typical range: 0-2500 ticks/sec (based on 158.2 PPR encoders, ~250 RPM max)
        target_velocity_a_ = msg->motor_speed_a;
        target_velocity_b_ = msg->motor_speed_b;

        // Warn if setpoint seems unusually high
        if (std::abs(target_velocity_a_) > 3000.0f || std::abs(target_velocity_b_) > 3000.0f) {
            RCLCPP_WARN(this->get_logger(),
                "High velocity setpoint: A=%.1f B=%.1f ticks/sec (max recommended: 2500)",
                target_velocity_a_, target_velocity_b_);
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

        float velocity_raw_a = (current_encoder_a - last_encoder_a_) / dt;
        float velocity_raw_b = (current_encoder_b - last_encoder_b_) / dt;

        last_encoder_a_ = current_encoder_a;
        last_encoder_b_ = current_encoder_b;

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

        // Auto-tune based on motor selection
        bool tune_a = (goal->motor_select == AutoTunePID::Goal::MOTOR_A ||
                       goal->motor_select == AutoTunePID::Goal::BOTH_MOTORS);
        bool tune_b = (goal->motor_select == AutoTunePID::Goal::MOTOR_B ||
                       goal->motor_select == AutoTunePID::Goal::BOTH_MOTORS);

        float Ku_a = 0, Tu_a = 0;
        float Ku_b = 0, Tu_b = 0;

        // Tune Motor A
        if (tune_a) {
            feedback->status = "Auto-tuning Motor A...";
            feedback->current_motor = AutoTunePID::Goal::MOTOR_A;
            feedback->progress = 0.0;
            goal_handle->publish_feedback(feedback);

            encoder_a_count_ = 0;
            last_encoder_a_ = 0;
            if (autoTuneMotor(1, goal->target_velocity, goal->max_duration, goal_handle, Ku_a, Tu_a)) {
                result->ku_a = Ku_a;
                result->tu_a = Tu_a;
                result->kp_a = pid_motor_a_.Kp;
                result->ki_a = pid_motor_a_.Ki;
                result->kd_a = pid_motor_a_.Kd;
                RCLCPP_INFO(this->get_logger(), "Motor A tuned: Ku=%.3f Tu=%.3f", Ku_a, Tu_a);
            } else {
                result->message = "Failed to tune Motor A";
                goal_handle->abort(result);
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
            if (autoTuneMotor(2, goal->target_velocity, goal->max_duration, goal_handle, Ku_b, Tu_b)) {
                result->ku_b = Ku_b;
                result->tu_b = Tu_b;
                result->kp_b = pid_motor_b_.Kp;
                result->ki_b = pid_motor_b_.Ki;
                result->kd_b = pid_motor_b_.Kd;
                RCLCPP_INFO(this->get_logger(), "Motor B tuned: Ku=%.3f Tu=%.3f", Ku_b, Tu_b);
            } else {
                result->message = "Failed to tune Motor B";
                goal_handle->abort(result);
                return;
            }
        }

        // Save gains to YAML
        std::string config_path = "/home/ubuntu/jerro_ws/src/jerro_drivers/config/pid_gains.yaml";
        saveGainsToYAML(config_path,
                        result->kp_a, result->ki_a, result->kd_a,
                        result->kp_b, result->ki_b, result->kd_b);

        // Success!
        result->success = true;
        result->message = "Auto-tuning completed successfully";
        feedback->status = "Auto-tuning completed";
        feedback->progress = 1.0;
        goal_handle->publish_feedback(feedback);
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Auto-tuning completed successfully");
    }

    bool autoTuneMotor(int motor_num, float target_velocity, float max_duration,
                       std::shared_ptr<GoalHandleAutoTune> goal_handle,
                       float& Ku_out, float& Tu_out)
    {
        using namespace std::chrono;

        // Get PID controller reference
        PIDController* pid = (motor_num == 1) ? &pid_motor_a_ : &pid_motor_b_;

        // Save original gains
        float original_Kp = pid->Kp;
        float original_Ki = pid->Ki;
        float original_Kd = pid->Kd;

        // Initialize for pure proportional control
        pid->Ki = 0.0f;
        pid->Kd = 0.0f;
        pid->Kp = Kp_start_;
        pid->reset();

        RCLCPP_INFO(this->get_logger(), "Auto-tuning motor %d: target=%.1f ticks/sec",
                    motor_num, target_velocity);

        auto start_time = steady_clock::now();

        // Sweep Kp
        while (pid->Kp <= Kp_max_) {
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                pid->setGains(original_Kp, original_Ki, original_Kd);
                stopMotors();
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

            OscillationDetector detector;
            pid->reset();

            auto test_start = steady_clock::now();
            auto last_time = test_start;

            int last_count = (motor_num == 1) ? encoder_a_count_.load() : encoder_b_count_.load();
            ExponentialFilter filter(0.2f);

            bool oscillation_found = false;
            float Tu_detected = 0;

            // Control loop for this Kp value
            while (rclcpp::ok()) {
                auto current_time = steady_clock::now();
                float elapsed = duration_cast<milliseconds>(current_time - test_start).count() / 1000.0f;

                // Check timeout
                if (elapsed > test_duration_per_Kp_) {
                    RCLCPP_INFO(this->get_logger(), "  No oscillation at Kp=%.3f", pid->Kp);
                    break;
                }

                // Calculate dt
                float dt = duration_cast<milliseconds>(current_time - last_time).count() / 1000.0f;
                last_time = current_time;

                if (dt < 0.001f || dt > 1.0f) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }

                // Measure velocity
                int current_count = (motor_num == 1) ? encoder_a_count_.load() : encoder_b_count_.load();
                float velocity_raw = (current_count - last_count) / dt;
                last_count = current_count;
                float velocity = filter.update(velocity_raw);

                // Compute PID
                float pwm = pid->compute(target_velocity, velocity, dt);
                setMotorPWM(motor_num, pwm);

                // Detect oscillation (after settling)
                if (elapsed > settling_time_) {
                    if (detector.detectOscillation(pid->raw_error, dt, Tu_detected)) {
                        oscillation_found = true;
                        Ku_out = pid->Kp;
                        Tu_out = Tu_detected;

                        RCLCPP_INFO(this->get_logger(), "  OSCILLATION DETECTED!");
                        RCLCPP_INFO(this->get_logger(), "  Ku = %.3f, Tu = %.3f sec", Ku_out, Tu_out);
                        break;
                    }
                }

                std::this_thread::sleep_for(20ms);
            }

            // Stop motor
            setMotorPWM(motor_num, 0);
            std::this_thread::sleep_for(500ms);

            if (oscillation_found) {
                // Success! Calculate Ziegler-Nichols gains
                pid->calculateZieglerNicholsGains(Ku_out, Tu_out);
                RCLCPP_INFO(this->get_logger(), "Motor %d tuned: Kp=%.3f Ki=%.3f Kd=%.3f",
                            motor_num, pid->Kp, pid->Ki, pid->Kd);
                return true;
            }

            // Increment Kp
            pid->Kp += Kp_increment_;

            // Check global timeout
            float total_elapsed = duration_cast<seconds>(steady_clock::now() - start_time).count();
            if (total_elapsed > max_duration) {
                RCLCPP_ERROR(this->get_logger(), "Auto-tuning timeout");
                pid->setGains(original_Kp, original_Ki, original_Kd);
                stopMotors();
                return false;
            }
        }

        // Failed to find oscillation
        RCLCPP_ERROR(this->get_logger(), "Failed to find oscillation for motor %d", motor_num);
        pid->setGains(original_Kp, original_Ki, original_Kd);
        stopMotors();
        return false;
    }

    void saveGainsToYAML(const std::string& filepath,
                         float Kp_a, float Ki_a, float Kd_a,
                         float Kp_b, float Ki_b, float Kd_b)
    {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", filepath.c_str());
            return;
        }

        file << std::fixed << std::setprecision(6);
        file << "# Auto-tuned PID gains\n";
        file << "# Generated by motor_pid_controller auto-tune action\n\n";
        file << "motor_pid_controller:\n";
        file << "  ros__parameters:\n";
        file << "    motor_a:\n";
        file << "      Kp: " << Kp_a << "\n";
        file << "      Ki: " << Ki_a << "\n";
        file << "      Kd: " << Kd_a << "\n";
        file << "    motor_b:\n";
        file << "      Kp: " << Kp_b << "\n";
        file << "      Ki: " << Ki_b << "\n";
        file << "      Kd: " << Kd_b << "\n";

        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved tuned gains to %s", filepath.c_str());
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
