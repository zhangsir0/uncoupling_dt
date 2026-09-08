#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class LidarTfPublisherNode : public rclcpp::Node {
public:
  LidarTfPublisherNode() : Node("lidar_tf_publisher_node",
    rclcpp::NodeOptions().use_intra_process_comms(true)) {

    // ===================== 关键修复：所有参数必须在这里先声明 =====================
    parent_frame_ = declare_parameter<std::string>("parent_frame", "hesai_lidar");
    rs1_frame_ = declare_parameter<std::string>("rs1_frame", "rslidar_1");
    rs2_frame_ = declare_parameter<std::string>("rs2_frame", "rslidar_2");
    fixposition_frame_ = declare_parameter<std::string>("fixposition_frame", "FP_VRTK");

    // 向量参数必须在构造函数一开始就声明！！！
    std::vector<double> default_rs1 = {1.11424, -0.342585, -0.0515785, -0.497064, 0.867635, 0.00538142, 0.0104393};
    std::vector<double> default_rs2 = {1.26219, -0.392196, -0.0448089, 0.860377, -0.50956, -0.00773477, -0.0063509};
    std::vector<double> default_fixposition = {1.65, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0};

    declare_parameter<std::vector<double>>("rs1_transform", default_rs1);
    declare_parameter<std::vector<double>>("rs2_transform", default_rs2);
    declare_parameter<std::vector<double>>("fixposition_transform", default_fixposition);

    // ==========================================================================

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 现在再读取参数
    initExtrinsics();
    publishStaticTransforms();

    RCLCPP_INFO(get_logger(),
      "Published static lidar TFs: %s -> %s, %s -> %s, %s -> %s",
      parent_frame_.c_str(), rs1_frame_.c_str(),
      parent_frame_.c_str(), rs2_frame_.c_str(),
      parent_frame_.c_str(), fixposition_frame_.c_str());
  }

private:
  Eigen::Affine3f buildTransform(const std::vector<double>& params) {
    if (params.size() != 7) {
      RCLCPP_ERROR(get_logger(), "Transform parameter must have 7 elements: x y z qx qy qz qw");
      return Eigen::Affine3f::Identity();
    }
    Eigen::Translation3f translation(params[0], params[1], params[2]);
    Eigen::Quaternionf rotation(params[6], params[3], params[4], params[5]); // qw, qx, qy, qz
    return translation * rotation;
  }

  void initExtrinsics() {
    // 这里只读取，不声明！
    std::vector<double> rs1_param = get_parameter("rs1_transform").as_double_array();
    std::vector<double> rs2_param = get_parameter("rs2_transform").as_double_array();
    std::vector<double> fixposition_param = get_parameter("fixposition_transform").as_double_array();

    transform_rs1_ = buildTransform(rs1_param);
    transform_rs2_ = buildTransform(rs2_param);
    transform_fixposition_ = buildTransform(fixposition_param);
  }

  geometry_msgs::msg::TransformStamped toTfMsg(
      const Eigen::Affine3f &tf,
      const std::string &parent_frame,
      const std::string &child_frame,
      const rclcpp::Time &stamp) const {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = child_frame;

    msg.transform.translation.x = tf.translation().x();
    msg.transform.translation.y = tf.translation().y();
    msg.transform.translation.z = tf.translation().z();

    Eigen::Quaternionf q(tf.rotation());
    q.normalize();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    return msg;
  }

  void publishStaticTransforms() {
    const auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(3);
    transforms.push_back(toTfMsg(transform_rs1_, parent_frame_, rs1_frame_, stamp));
    transforms.push_back(toTfMsg(transform_rs2_, parent_frame_, rs2_frame_, stamp));
    transforms.push_back(
        toTfMsg(transform_fixposition_, parent_frame_, fixposition_frame_, stamp));
    static_tf_broadcaster_->sendTransform(transforms);
  }

private:
  std::string parent_frame_;
  std::string rs1_frame_;
  std::string rs2_frame_;
  std::string fixposition_frame_;

  Eigen::Affine3f transform_rs1_{Eigen::Affine3f::Identity()};
  Eigen::Affine3f transform_rs2_{Eigen::Affine3f::Identity()};
  Eigen::Affine3f transform_fixposition_{Eigen::Affine3f::Identity()};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTfPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
