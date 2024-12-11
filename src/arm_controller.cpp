#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "tmotor.hpp"

class JoySubscriber : public rclcpp::Node
{
public:
  JoySubscriber() : Node("joy_subscriber"), base_speed_(100.0f), prev_dpad_state_(0)
  {
    motor_cross_.setMotorID(12);
    motor_circle_.setMotorID(2);
    motor_square_.setMotorID(3);
    motor_triangle_.setMotorID(4);

    motor_cross_.connect("can0");
    motor_circle_.connect("can0");
    motor_square_.connect("can0");
    motor_triangle_.connect("can0");

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoySubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  const int CROSS_BTN = 1;
  const int CIRCLE_BTN = 2;
  const int SQUARE_BTN = 0;
  const int TRIANGLE_BTN = 3;
  const int L1_BTN = 4;
  const int L2_AXIS = 3;

  // Axis for D-pad vertical (adjust as necessary)
  const int DPAD_VERTICAL_AXIS = 7; 
  // Threshold to consider pressed vs neutral
  const float PRESS_THRESHOLD = 0.5f;

  // For R2 axis (future use)
  const int R2_AXIS = 4; // Just an example, adjust to your controller

  float base_speed_;
  int prev_dpad_state_; // -1 for up pressed, 0 for neutral, +1 for down pressed

  void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    bool l2_pressed = false;
    if ((int)msg->axes.size() > L2_AXIS && msg->axes[L2_AXIS] < 0.0) {
      l2_pressed = true;
    }

    bool cross_pressed = ((int)msg->buttons.size() > CROSS_BTN) && (msg->buttons[CROSS_BTN] == 1);
    bool circle_pressed = ((int)msg->buttons.size() > CIRCLE_BTN) && (msg->buttons[CIRCLE_BTN] == 1);
    bool square_pressed = ((int)msg->buttons.size() > SQUARE_BTN) && (msg->buttons[SQUARE_BTN] == 1);
    bool triangle_pressed = ((int)msg->buttons.size() > TRIANGLE_BTN) && (msg->buttons[TRIANGLE_BTN] == 1);
    bool l1_pressed = ((int)msg->buttons.size() > L1_BTN) && (msg->buttons[L1_BTN] == 1);

    float r2_value = 0.0f;
    if ((int)msg->axes.size() > R2_AXIS) {
      r2_value = msg->axes[R2_AXIS];
      // Future use for r2_value
    }

    // Determine current D-pad vertical state
    int dpad_state = 0;
    if ((int)msg->axes.size() > DPAD_VERTICAL_AXIS) {
      float dpad_y = msg->axes[DPAD_VERTICAL_AXIS];
      RCLCPP_INFO(this->get_logger(), "dpad_y: %.2f", dpad_y);
      if (dpad_y > PRESS_THRESHOLD) {
        dpad_state = -1; // Up pressed
      } else if (dpad_y < -PRESS_THRESHOLD) {
        dpad_state = +1; // Down pressed
      } else {
        dpad_state = 0; // Neutral
      }
    }
    

    // Only adjust speed on a transition from neutral to pressed
    // i.e., if current state differs from previous, and current != 0
    if (dpad_state != prev_dpad_state_ && dpad_state != 0) {
      // A new press detected
      if (dpad_state == -1) {
        // Up pressed: increase speed by 10
        base_speed_ += 10.0f;
        if (base_speed_ > 300.0f) {
          base_speed_ = 300.0f;
        }
      } else if (dpad_state == 1) {
        // Down pressed: decrease speed by 10
        base_speed_ -= 10.0f;
        if (base_speed_ < 0.0f) {
          base_speed_ = 0.0f;
        }
      }
    }

    // Update previous D-pad state
    prev_dpad_state_ = dpad_state;

    // Determine final command
    float command = base_speed_;
    if (l1_pressed) {
      command = -base_speed_;
    }

    bool any_face_button = cross_pressed || circle_pressed || square_pressed || triangle_pressed;

    try {
      if (l2_pressed && any_face_button) {
        if (cross_pressed) {
          motor_cross_.sendVelocity(command);
        } else {
          motor_cross_.sendVelocity(0);
        }
        if (circle_pressed) {
          motor_circle_.sendVelocity(command);
        } else {
          motor_circle_.sendVelocity(0);
        }
        if (square_pressed) {
          motor_square_.sendVelocity(command);
        } else {
          motor_square_.sendVelocity(0);
        }
        if (triangle_pressed) {
          motor_triangle_.sendVelocity(command);
        } else {
          motor_triangle_.sendVelocity(0);
        }
      } else {
        // Conditions not met, send 0 velocity to all
        motor_cross_.sendVelocity(0);
        motor_circle_.sendVelocity(0);
        motor_square_.sendVelocity(0);
        motor_triangle_.sendVelocity(0);
      }
    } catch (const TMotor::CANSocketException &e) {
      RCLCPP_ERROR(this->get_logger(), "Error sending velocity to motor: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Base speed: %.2f, R2: %.2f", base_speed_, r2_value);
    
  }
  

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

  TMotor::AKManager motor_cross_;
  TMotor::AKManager motor_circle_;
  TMotor::AKManager motor_square_;
  TMotor::AKManager motor_triangle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoySubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
