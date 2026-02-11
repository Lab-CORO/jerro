#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>
#include <memory>
#include <iostream>
#include "jerro_msgs/srv/set_servo_pos.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Configuration servo
const int SERVO_PIN = 25;           // GPIO 25 (broche 22)
const int SERVO_MIN_PULSE = 1000;   // 1 ms (1000 µs) - position minimale
const int SERVO_MAX_PULSE = 2000;   // 2 ms (2000 µs) - position maximale
const int SERVO_CENTER_PULSE = 1500; // 1.5 ms - position centrale
// Note: set_servo_pulsewidth generates ~50Hz PWM automatically

// Conversion pulse (µs) -> angle (rad) pour le joint
const double SERVO_START_ANGLE = 0.623334;       // Angle (rad) a SERVO_MAX_PULSE
const double SERVO_INC_PER_US  = -0.001558335;   // Radian par microseconde
const std::string JOINT_NAME   = "base_link_to_servomotor";

class ServomoteurNode : public rclcpp::Node
{
public:
    ServomoteurNode() : Node("servomoteur")
    {
        // Initialize pigpio
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon");
            throw std::runtime_error("pigpio initialization failed");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to pigpio daemon");

        // Initialize servo at center position using set_servo_pulsewidth
        // This function is specifically designed for servos and works on any GPIO
        current_pulse_width_ = SERVO_CENTER_PULSE;
        int ret = set_servo_pulsewidth(pi_, SERVO_PIN, current_pulse_width_);

        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Servo initialized on GPIO %d at %d µs (software PWM)",
                       SERVO_PIN, current_pulse_width_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo (ret=%d)", ret);
            throw std::runtime_error("servo initialization failed");
        }

        // Create JointState publisher
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/set_joint_states", 10);
        publishJointState();

        // Create ROS2 service
        service_set_pos_ = this->create_service<jerro_msgs::srv::SetServoPos>(
            "set_servo_pos",
            std::bind(&ServomoteurNode::setServoPos, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Servomoteur node ready (GPIO 25, software PWM)");
        RCLCPP_INFO(this->get_logger(), "Service: /set_servo_pos");
        RCLCPP_INFO(this->get_logger(), "Position range: %d-%d µs", SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    }

    ~ServomoteurNode()
    {
        // Stop servo (pulsewidth=0 disables the servo)
        set_servo_pulsewidth(pi_, SERVO_PIN, 0);
        RCLCPP_INFO(this->get_logger(), "Servo stopped");

        // Close pigpio
        if (pi_ >= 0) {
            pigpio_stop(pi_);
            RCLCPP_INFO(this->get_logger(), "Pigpio stopped");
        }
    }

private:
    int pi_;
    int current_pulse_width_;  // Pulse width in microseconds (1000-2000)

    rclcpp::Service<jerro_msgs::srv::SetServoPos>::SharedPtr service_set_pos_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    void publishJointState()
    {
        double angle = SERVO_START_ANGLE
            + (SERVO_MAX_PULSE - current_pulse_width_) * SERVO_INC_PER_US;

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();
        msg.name.push_back(JOINT_NAME);
        msg.position.push_back(angle);
        joint_state_pub_->publish(msg);
    }

    void setServoPulseWidth(int pulse_us)
    {
        // Clamp pulse width to valid range
        if (pulse_us < SERVO_MIN_PULSE) pulse_us = SERVO_MIN_PULSE;
        if (pulse_us > SERVO_MAX_PULSE) pulse_us = SERVO_MAX_PULSE;

        current_pulse_width_ = pulse_us;

        // set_servo_pulsewidth generates ~50Hz PWM signal automatically
        int ret = set_servo_pulsewidth(pi_, SERVO_PIN, pulse_us);

        if (ret == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Servo position: %d µs", pulse_us);
            publishJointState();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set servo position (ret=%d)", ret);
        }
    }

    // Service callback: Set servo position
    // Request.position is in microseconds (1000-2000 µs)
    void setServoPos(
        const std::shared_ptr<jerro_msgs::srv::SetServoPos::Request> request,
        std::shared_ptr<jerro_msgs::srv::SetServoPos::Response> response)
    {
        int pulse_us = request->position;

        // Validate range
        if (pulse_us < SERVO_MIN_PULSE || pulse_us > SERVO_MAX_PULSE) {
            RCLCPP_WARN(this->get_logger(),
                       "Invalid servo position %d µs (valid: %d-%d µs)",
                       pulse_us, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
            response->result = false;
        } else {
            setServoPulseWidth(pulse_us);
            response->result = true;
            RCLCPP_INFO(this->get_logger(), "Servo moved to %d µs", pulse_us);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ServomoteurNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
