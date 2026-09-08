#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "magnetic_guide_driver/msg/magnetic_data.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"

#include <cmath>
#include <algorithm>
#include <mutex>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MagneticGuideController : public rclcpp::Node
{
public:
  MagneticGuideController()
  : Node("magnetic_guide_controller_node"),
    last_error_(0.0),
    integral_(0.0),
    current_gear_(3) // 初始化为 N 档
  {
    // 声明参数
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("ki", 0.01);
    this->declare_parameter("kd", 0.1);
    
    this->declare_parameter("safe_sum_threshold", 800.0);
    this->declare_parameter("diff_normalize_range", 500.0);
    
    this->declare_parameter("max_steering_angle", 0.4712);
    this->declare_parameter("max_speed_forward", 1.0);
    this->declare_parameter("min_speed_forward", 0.3);
    this->declare_parameter("max_speed_reverse", 0.5);
    this->declare_parameter("min_speed_reverse", 0.15);

    this->declare_parameter("front_magnetic_topic", "/magnetic_guide_data_1");
    this->declare_parameter("rear_magnetic_topic", "/magnetic_guide_data_2");
    this->declare_parameter("chassis_fb_topic", "/chassis_info_fb");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("ackermann_topic", "/ackermann_cmd");

    this->declare_parameter("force_gear", 0); // 0表示订阅真实反馈，4表示强制D档，2表示强制R档
    this->declare_parameter("reverse_forward_steering", false);

    // 获取参数
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    safe_sum_threshold_ = this->get_parameter("safe_sum_threshold").as_double();
    diff_normalize_range_ = this->get_parameter("diff_normalize_range").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_speed_forward_ = this->get_parameter("max_speed_forward").as_double();
    min_speed_forward_ = this->get_parameter("min_speed_forward").as_double();
    max_speed_reverse_ = this->get_parameter("max_speed_reverse").as_double();
    min_speed_reverse_ = this->get_parameter("min_speed_reverse").as_double();

    force_gear_ = this->get_parameter("force_gear").as_int();
    reverse_forward_steering_ = this->get_parameter("reverse_forward_steering").as_bool();

    std::string front_topic = this->get_parameter("front_magnetic_topic").as_string();
    std::string rear_topic = this->get_parameter("rear_magnetic_topic").as_string();
    std::string chassis_topic = this->get_parameter("chassis_fb_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    std::string ackermann_topic = this->get_parameter("ackermann_topic").as_string();

    // 订阅 底盘档位信息
    chassis_sub_ = this->create_subscription<yhs_can_interfaces::msg::ChassisInfoFb>(
      chassis_topic, 10, std::bind(&MagneticGuideController::chassis_callback, this, _1));

    // 订阅 磁导航 数据 (前传感器与后传感器)
    front_mag_sub_ = this->create_subscription<magnetic_guide_driver::msg::MagneticData>(
      front_topic, 10, std::bind(&MagneticGuideController::front_topic_callback, this, _1));
      
    rear_mag_sub_ = this->create_subscription<magnetic_guide_driver::msg::MagneticData>(
      rear_topic, 10, std::bind(&MagneticGuideController::rear_topic_callback, this, _1));

    // 发布指令
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);
    
    // 定时器控制环路 (由各传感器回调驱动代替定时器，避免无数据时空转)
  }

private:
  void chassis_callback(const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_gear_ = msg->ctrl_fb.ctrl_fb_gear;
    // 01: P, 02: R, 03: N, 04: D
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "Received Chassis Gear: %d", current_gear_);
  }

  void front_topic_callback(const magnetic_guide_driver::msg::MagneticData::SharedPtr msg)
  {
    uint8_t gear = 3;
    if (force_gear_ != 0) {
      gear = force_gear_;
    } else {
      std::lock_guard<std::mutex> lock(state_mutex_);
      gear = current_gear_;
    }
    
    // D档才处理前传感器
    if (gear == 4) {
      process_control_logic(msg, true); // true表示前进
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Front sensor ticked, but NOT in Drive Gear! (Current Gear: %d)", gear);
    }
  }
  
  void rear_topic_callback(const magnetic_guide_driver::msg::MagneticData::SharedPtr msg)
  {
    uint8_t gear = 3;
    if (force_gear_ != 0) {
      gear = force_gear_;
    } else {
      std::lock_guard<std::mutex> lock(state_mutex_);
      gear = current_gear_;
    }

    // R档才处理后传感器
    if (gear == 2) {
      process_control_logic(msg, false); // false表示倒车
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Rear sensor ticked, but NOT in Reverse Gear! (Current Gear: %d)", gear);
    }
  }

  void process_control_logic(const magnetic_guide_driver::msg::MagneticData::SharedPtr msg, bool is_forward)
  {
    double current_error_raw = 0.0;
    bool track_detected = false;
    double max_sum = 0.0;

    // 1. 状态检测 (Failsafe)
    for (size_t i = 0; i < 6; ++i) {
      max_sum = std::max(max_sum, msg->sum[i]);
      if (msg->sum[i] > safe_sum_threshold_) {
        current_error_raw = msg->diff[i];
        track_detected = true;
        break; // 只要第一个满足条件的即可
      }
    }

    if (!track_detected) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "Track lost! (Max sum was %.1f < %.1f). Stopping vehicle.", max_sum, safe_sum_threshold_);
      publish_stop_command();
      integral_ = 0.0;
      last_error_ = 0.0;
      return;
    }

    // 归一化处理 diff 到 [-1.0, 1.0] 的偏差空间
    double normalized_diff = current_error_raw / diff_normalize_range_;
    normalized_diff = std::clamp(normalized_diff, -1.0, 1.0);

    // 依据前后传感器特征，得出 e(t):
    // "对前面的传感器车辆靠右diff为负值，靠左diff为正，中间为0，后向传感器相反"
    // 假设正的 e(t) 意味着车偏左，需要提供负角度（向右打轮）；负的 e(t) 表示车偏右，需要正角度（向左打轮）
    double current_error = is_forward ? normalized_diff : -normalized_diff;

    // 2. PID 计算 (PID Control)
    integral_ += current_error;
    double derivative = current_error - last_error_;
    last_error_ = current_error;

    // 积分限幅
    if (integral_ > 50.0) integral_ = 50.0;
    if (integral_ < -50.0) integral_ = -50.0;

    double p_out = kp_ * current_error;
    double i_out = ki_ * integral_;
    double d_out = kd_ * derivative;

    // 前进基础 PID 方向计算结果
    // 这里的PID参数需自己调一下符号，如果打角反了可以把 P I D 填成负的
    double raw_steering = p_out + i_out + d_out;

    // 倒车打角反转(Direction)
    double direction = is_forward ? (reverse_forward_steering_ ? -1.0 : 1.0) : -1.0;
    double target_steering_angle = raw_steering * direction;

    // 3. 输出限幅与速度联动
    target_steering_angle = std::clamp(target_steering_angle, -max_steering_angle_, max_steering_angle_);

    double steering_ratio = std::abs(target_steering_angle) / max_steering_angle_; 
    
    double max_speed = is_forward ? max_speed_forward_ : max_speed_reverse_;
    double min_speed = is_forward ? min_speed_forward_ : min_speed_reverse_;
    
    // 直道(转角小)快，弯道(转角大)慢
    double target_speed_mag = max_speed - steering_ratio * (max_speed - min_speed);
    
    // 结合具体的前进方向，倒车时线速度给负值
    double target_speed = is_forward ? target_speed_mag : -target_speed_mag;

    // 4. 发送指令
    publish_control_command(target_speed, target_steering_angle);

    RCLCPP_INFO(this->get_logger(), "Mode:%s | RawDiff:%.1f | Err:%.2f | Ang:%.3f | Spd:%.2f", 
      is_forward ? "FWD" : "REV", current_error_raw, current_error, target_steering_angle, target_speed);
  }

  void publish_control_command(double speed, double steering_angle)
  {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = steering_angle; 
    twist_publisher_->publish(twist_msg);

    auto ack_msg = ackermann_msgs::msg::AckermannDriveStamped();
    ack_msg.header.stamp = this->now();
    ack_msg.header.frame_id = "base_link"; 
    ack_msg.drive.speed = 0.0;
    // 有的底盘接收 Ackermann 可能会限制 steering_angle 只有固定正负判定规则，请确保物理极性
    ack_msg.drive.steering_angle = steering_angle;
    ack_msg.drive.steering_angle_velocity = 0.0; 
    ackermann_publisher_->publish(ack_msg);
  }

  void publish_stop_command()
  {
    publish_control_command(0.0, 0.0);
  }

  // 参数变量
  double kp_, ki_, kd_;
  double safe_sum_threshold_, diff_normalize_range_;
  double max_steering_angle_;
  double max_speed_forward_, min_speed_forward_;
  double max_speed_reverse_, min_speed_reverse_;

  // PID 变量
  double last_error_;
  double integral_;
  
  // 状态变量
  uint8_t current_gear_;
  int force_gear_;
  bool reverse_forward_steering_;
  std::mutex state_mutex_;

  // 节点组件
  rclcpp::Subscription<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr chassis_sub_;
  rclcpp::Subscription<magnetic_guide_driver::msg::MagneticData>::SharedPtr front_mag_sub_;
  rclcpp::Subscription<magnetic_guide_driver::msg::MagneticData>::SharedPtr rear_mag_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MagneticGuideController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}