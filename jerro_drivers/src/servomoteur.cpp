#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <cstdlib>  // pour system()
#include <iostream>
#include "jerro_msgs/srv/set_servo_pos.hpp"
#include "jerro_msgs/srv/toggle_servomotor.hpp"


void pwm_wheel_to_servo() {
    system("gpio write 25 0");
    system("gpio write 3 0");
    system("gpio write 27 0");
    system("gpio mode 1 pwm");
    system("gpio pwmc 192");
    system("gpio pwm-ms");
}

void pwm_servo_to_wheel() {
    system("gpio write 25 0");
    system("gpio write 3 0");
    system("gpio write 27 0");
    system("gpio pwm 1 0");
    system("gpio mode 1 in");
    system("gpio mode 26 pwm");
    system("gpio mode 23 pwm");
    system("gpio pwm-bal");
    system("gpio pwm 26 512");
    system("gpio pwm 23 512");
    system("gpio write 25 1");
    system("gpio write 3 1");
    system("gpio write 27 1");
}


uint8_t state = jerro_msgs::srv::ToggleServomotor::Request::OFF;

void set_servo_pos(const std::shared_ptr<jerro_msgs::srv::SetServoPos::Request> request,
          std::shared_ptr<jerro_msgs::srv::SetServoPos::Response> response)
{
    if(state == jerro_msgs::srv::ToggleServomotor::Request::ON){
        int servo_pos = request->position;
        std::string command = "gpio pwm 1 " + std::to_string(servo_pos);
        system(command.c_str());
        response->result = true;
    }else{
        response->result = false;
    }
    sleep(1);

}

void toggle_servomotor(const std::shared_ptr<jerro_msgs::srv::ToggleServomotor::Request> request,
          std::shared_ptr<jerro_msgs::srv::ToggleServomotor::Response> response)
{
    if(request->state == jerro_msgs::srv::ToggleServomotor::Request::ON){
        pwm_wheel_to_servo();
        response->result = true;

    }else if (request->state == jerro_msgs::srv::ToggleServomotor::Request::OFF){
        pwm_servo_to_wheel();
        response->result = true;
    }else{
        response->result = false;
    }
    state = request->state;
    sleep(1);

}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("servomoteur");

  rclcpp::Service<jerro_msgs::srv::SetServoPos>::SharedPtr service_set_pos =
    node->create_service<jerro_msgs::srv::SetServoPos>("set_servo_pos", &set_servo_pos);

  rclcpp::Service<jerro_msgs::srv::ToggleServomotor>::SharedPtr service_state_motor =
    node->create_service<jerro_msgs::srv::ToggleServomotor>("toggle_servomotor", &toggle_servomotor);


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready move servo.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}