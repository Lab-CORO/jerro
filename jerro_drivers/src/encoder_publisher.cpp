#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pigpiod_if2.h>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

// Inversion encodeurs si nécessaire (comme dans pid_control.cpp)
static constexpr int ENCODER_A_INVERT = -1;  // Inversé pour correspondre au sens du moteur
static constexpr int ENCODER_B_INVERT = 1;  // Inversé (ligne 32 pid_control.cpp)

// Transit table for quadrature decoding (from pid_control.cpp)
static int transits[16] = {
    0,  -1,   1,   0,   1,   0,   0,  -1,
   -1,   0,   0,   1,   0,   1,  -1,   0
};

class EncoderPublisher : public rclcpp::Node
{
public:
    EncoderPublisher()
    : Node("encoder_publisher")
    {
        // Initialize hardware
        initPigpio();
        setupEncoders();

        // Create publishers
        pub_encoder_a_ = this->create_publisher<std_msgs::msg::Int32>("/encoder_a", 10);
        pub_encoder_b_ = this->create_publisher<std_msgs::msg::Int32>("/encoder_b", 10);

        // Create timer for 50Hz publication
        timer_ = this->create_wall_timer(
            20ms, std::bind(&EncoderPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Encoder publisher initialized");
    }

    ~EncoderPublisher()
    {
        if (pi_ >= 0) {
            pigpio_stop(pi_);
        }
    }

private:
    // Hardware management
    int pi_;

    // GPIO pins
    static constexpr int S1A = 23;
    static constexpr int S1B = 24;
    static constexpr int S2A = 17;
    static constexpr int S2B = 27;

    // Encoder state (atomic for thread safety from GPIO callbacks)
    std::atomic<int> encoder_a_count_{0};
    std::atomic<int> encoder_b_count_{0};
    std::atomic<int> lev_a1_{0}, lev_a2_{0};
    std::atomic<int> lev_b1_{0}, lev_b2_{0};
    std::atomic<int> old_state_a_{0};
    std::atomic<int> old_state_b_{0};

    // ROS2 components
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_encoder_a_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_encoder_b_;
    rclcpp::TimerBase::SharedPtr timer_;

    void initPigpio()
    {
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon");
            throw std::runtime_error("pigpio initialization failed");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to pigpio daemon");
    }

    void setupEncoders()
    {
        // Configure Motor A encoder pins
        set_mode(pi_, S1A, PI_INPUT);
        set_mode(pi_, S1B, PI_INPUT);
        set_pull_up_down(pi_, S1A, PI_PUD_UP);
        set_pull_up_down(pi_, S1B, PI_PUD_UP);
        set_glitch_filter(pi_, S1A, 1000);
        set_glitch_filter(pi_, S1B, 1000);

        // Configure Motor B encoder pins
        set_mode(pi_, S2A, PI_INPUT);
        set_mode(pi_, S2B, PI_INPUT);
        set_pull_up_down(pi_, S2A, PI_PUD_UP);
        set_pull_up_down(pi_, S2B, PI_PUD_UP);
        set_glitch_filter(pi_, S2A, 1000);
        set_glitch_filter(pi_, S2B, 1000);

        // Read initial state
        lev_a1_ = gpio_read(pi_, S1A);
        lev_a2_ = gpio_read(pi_, S1B);
        old_state_a_ = (lev_a1_ << 1) | lev_a2_;

        lev_b1_ = gpio_read(pi_, S2A);
        lev_b2_ = gpio_read(pi_, S2B);
        old_state_b_ = (lev_b1_ << 1) | lev_b2_;

        // Register callbacks with 'this' as userdata
        callback_ex(pi_, S1A, EITHER_EDGE, encoderACallbackStatic, this);
        callback_ex(pi_, S1B, EITHER_EDGE, encoderACallbackStatic, this);
        callback_ex(pi_, S2A, EITHER_EDGE, encoderBCallbackStatic, this);
        callback_ex(pi_, S2B, EITHER_EDGE, encoderBCallbackStatic, this);

        RCLCPP_INFO(this->get_logger(), "Encoders configured");
    }

    void timerCallback()
    {
        // Publish encoder counts
        auto msg_a = std_msgs::msg::Int32();
        msg_a.data = encoder_a_count_.load();
        pub_encoder_a_->publish(msg_a);

        auto msg_b = std_msgs::msg::Int32();
        msg_b.data = encoder_b_count_.load();
        pub_encoder_b_->publish(msg_b);
    }

    // Static callback wrappers for pigpio C-style callbacks
    static void encoderACallbackStatic(int pi, unsigned gpio, unsigned level,
                                       uint32_t tick, void* user)
    {
        auto* self = static_cast<EncoderPublisher*>(user);
        self->handleEncoderA(gpio, level);
    }

    static void encoderBCallbackStatic(int pi, unsigned gpio, unsigned level,
                                       uint32_t tick, void* user)
    {
        auto* self = static_cast<EncoderPublisher*>(user);
        self->handleEncoderB(gpio, level);
    }

    // Actual callback implementations
    void handleEncoderA(unsigned gpio, unsigned level)
    {
        if (gpio == S1A) {
            lev_a1_ = level;
        } else if (gpio == S1B) {
            lev_a2_ = level;
        }

        int newState = (lev_a1_ << 1) | lev_a2_;
        int inc = transits[(old_state_a_ << 2) | newState];

        if (inc) {
            old_state_a_ = newState;
            encoder_a_count_ += inc * ENCODER_A_INVERT;
        }
    }

    void handleEncoderB(unsigned gpio, unsigned level)
    {
        if (gpio == S2A) {
            lev_b1_ = level;
        } else if (gpio == S2B) {
            lev_b2_ = level;
        }

        int newState = (lev_b1_ << 1) | lev_b2_;
        int inc = transits[(old_state_b_ << 2) | newState];

        if (inc) {
            old_state_b_ = newState;
            encoder_b_count_ += inc * ENCODER_B_INVERT;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<EncoderPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("encoder_publisher"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
