#define keyboardDebug

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "mavros_msgs/msg/state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "comp_vision/comp_vision.hpp"

#ifdef keyboardDebug
#include "keyboard_msgs/msg/key.hpp"
#endif

#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

/*бля это пиздец ебучий питон отравил мой навык програмирования и теперь я пишу как ебанат*/

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      stateSub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/uav1/mavros/state", 10, std::bind(&MinimalPublisher::getState, this, std::placeholders::_1));

      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      poseSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/uav1/mavros/local_position/pose", qos, std::bind(&MinimalPublisher::getPose, this, std::placeholders::_1));
      imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/uav1/mavros/Imu/data", 10, std::bind(&MinimalPublisher::getImu, this, std::placeholders::_1));
      imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/uav1/camera", 10, std::bind(&MinimalPublisher::getImage, this, std::placeholders::_1));
      imageDownSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/uav1/camera_down", 10, std::bind(&MinimalPublisher::getImageDown, this, std::placeholders::_1));

      #ifdef keyboardDebug
      keySubDown_ = this->create_subscription<keyboard_msgs::msg::Key>(
        "/keydown", 10, std::bind(&MinimalPublisher::getKeyDown, this, std::placeholders::_1));
      keySubUp_ = this->create_subscription<keyboard_msgs::msg::Key>(
        "/keyup", 10, std::bind(&MinimalPublisher::getKeyUp, this, std::placeholders::_1));
      #endif
      
      mainLoopTimer_ = this->create_wall_timer(
      20ms, std::bind(&MinimalPublisher::main_loop, this));
      mainLoopTimer_->cancel();
      takeoffTimer_ = this->create_wall_timer(
        100ms, std::bind(&MinimalPublisher::takeoff, this));

      modeClient_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
      armClient_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");
      paramClient_ = this->create_client<rcl_interfaces::srv::SetParameters>("/uav1/mavros/setpoint_velocity/set_parameters");
    
      posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
      velPub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/uav1/mavros/setpoint_velocity/cmd_vel", 10);
    }

  private:
    void main_loop()
    {
      auto twist = getKeyBoardTwist(1, 1);
      twist->twist.linear.z = getHightCorrectedZVelocity(2); 
      velPub_->publish(*twist);
      DF::CompVision::setImage(DF::CompVision::convertToMatFromSensorMsgs(imageDown_));
      DF::CompVision::setImage(DF::CompVision::findByColorRange(DF::CompVision::getImage(), cv::Scalar(0, 0, 0), cv::Scalar(20, 20,20)));
      DF::CompVision::camOn(DF::CompVision::getImage());
    }

    void takeoff(){
      if (modeClient_->wait_for_service() and mavrosState_->mode != "OFFBOARD"){
        auto modeReq = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        modeReq->base_mode = 0;
        modeReq->custom_mode = "OFFBOARD";

        auto futureMode = modeClient_->async_send_request(modeReq);
        count_++;
        RCLCPP_INFO(this->get_logger(), "Set Board Mode");
        //return;
      }
      
      if (armClient_->wait_for_service() and mavrosState_->armed != true){
        auto armReq = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        armReq->value = true;

        auto futureArm = armClient_->async_send_request(armReq);
        count_++;
        RCLCPP_INFO(this->get_logger(), "Set Arm");
        //return;
      }

        auto poseMsg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        poseMsg->pose.position.x = 0;
        poseMsg->pose.position.y = 0;
        poseMsg->pose.position.z = 2;

        posePub_->publish(*poseMsg);
        count_++;
        if (count_ > 100){
          takeoffTimer_->cancel();
          mainLoopTimer_->reset();
          RCLCPP_INFO(this->get_logger(), "timer canceled");
        }

        auto paramReq = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param;
        param.name = "mav_frame";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        param.value.integer_value = 1;
        paramReq->parameters.push_back(param);
        paramClient_->async_send_request(paramReq);
    }

    void getState(mavros_msgs::msg::State::SharedPtr msg){
      mavrosState_ = msg;
    }
    void getPose(geometry_msgs::msg::PoseStamped::SharedPtr msg){
      pose_ = msg;
    }
    void getImu(sensor_msgs::msg::Imu::SharedPtr msg){
      imu_ = msg;
    }
    void getImage(sensor_msgs::msg::Image::SharedPtr msg){
      image_ = msg;
    }
    void getImageDown(sensor_msgs::msg::Image::SharedPtr msg){
      imageDown_ = msg;
    }

    float getHightCorrectedZVelocity(float TargetHight){
      return -(pose_->pose.position.z - TargetHight) * 2;
    }


    #ifdef keyboardDebug
    bool keyboardState[6] = {false, false, false, false, false, false};
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keySubDown_;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keySubUp_;

    void getKeyDown(keyboard_msgs::msg::Key::SharedPtr msg){
      RCLCPP_INFO(this->get_logger(), "Key Pressed %i", msg->code);
      switch(msg->code){
        case 119:
          keyboardState[0] = true;
          break;
        case 97:
          keyboardState[1] = true;
          break;
        case 115:
          keyboardState[2] = true;
          break;
        case 100:
          keyboardState[3] = true;
          break;
        case 113:
          keyboardState[4] = true;
          break;
        case 101:
          keyboardState[5] = true;
          break;
        default:
          break;
      }
    }

    void getKeyUp(keyboard_msgs::msg::Key::SharedPtr msg){
      RCLCPP_INFO(this->get_logger(), "Key Released %i", msg->code);
      switch(msg->code){
        case 119:
          keyboardState[0] = false;
          break;
        case 97:
          keyboardState[1] = false;
          break;
        case 115:
          keyboardState[2] = false;
          break;
        case 100:
          keyboardState[3] = false;
          break;
        case 113:
          keyboardState[4] = false;
          break;
        case 101:
          keyboardState[5] = false;
          break;
        default:
          break;
      }
    }

    geometry_msgs::msg::TwistStamped::SharedPtr getKeyBoardTwist(float speed, float turn_speed){
      auto twistMsg = std::make_shared<geometry_msgs::msg::TwistStamped>();
      twistMsg->twist.linear.x = 0;
      twistMsg->twist.linear.y = 0;
      twistMsg->twist.linear.z = 0;
      twistMsg->twist.angular.x = 0;
      twistMsg->twist.angular.y = 0;
      twistMsg->twist.angular.z = 0;
      
      twistMsg->header.frame_id = "BODY_NED";  // Используем frame_id "BODY_NED"
      twistMsg->header.stamp = this->now();

      if (keyboardState[0]){
        twistMsg->twist.linear.x = speed;
      }
      if (keyboardState[1]){
        twistMsg->twist.linear.y = speed;
      }
      if (keyboardState[2]){
        twistMsg->twist.linear.x = -speed;
      }
      if (keyboardState[3]){
        twistMsg->twist.linear.y = -speed;
      }
      if (keyboardState[4]){
        twistMsg->twist.angular.z = turn_speed;
      }
      if (keyboardState[5]){
        twistMsg->twist.angular.z = -turn_speed;
      }

      return twistMsg;
    }
    #endif

    rclcpp::TimerBase::SharedPtr mainLoopTimer_;
    rclcpp::TimerBase::SharedPtr takeoffTimer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageDownSub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr modeClient_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr armClient_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr paramClient_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velPub_;
    
    mavros_msgs::msg::State::SharedPtr mavrosState_ = std::make_shared<mavros_msgs::msg::State>();

    geometry_msgs::msg::PoseStamped::SharedPtr pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

    sensor_msgs::msg::Imu::SharedPtr imu_ = std::make_shared<sensor_msgs::msg::Imu>();
    sensor_msgs::msg::Image::SharedPtr image_ = std::make_shared<sensor_msgs::msg::Image>();
    sensor_msgs::msg::Image::SharedPtr imageDown_ = std::make_shared<sensor_msgs::msg::Image>();

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr nodeBaseInterface_ = this->get_node_base_interface();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}