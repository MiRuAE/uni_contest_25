// #include <cmath>
// #include <iostream>
// #include <memory>
// #include <sstream>
// #include <string>
// #include <thread>
// #include <mutex>
// #include <Eigen/Dense>

// #include "rclcpp/rclcpp.hpp"
// #include "vesc_msgs/msg/vesc_state_stamped.hpp"
// #include "vesc_msgs/msg/vesc_imu_stamped.hpp"
// #include "odom_msgs/msg/my_odom.hpp"
// #include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
// #include "std_msgs/msg/float64.hpp"

// using std::placeholders::_1;

// namespace vesc_ackermann {

// // New EKF class to estimate [x, y, theta, v]
// class EKF {
// public:
//   EKF() {
//     x_ = Eigen::Vector4d::Zero(); // [x, y, theta, v]
//     P_ = Eigen::Matrix4d::Identity() * 0.1;
//   }

//   void predict(double dt, double steering_angle, double wheelbase) {
//     double theta = x_(2);
//     double v = x_(3);

//     double w = v * tan(steering_angle) / wheelbase;

//     Eigen::Vector4d x_pred;
//     x_pred(0) = x_(0) + v * cos(theta) * dt;
//     x_pred(1) = x_(1) + v * sin(theta) * dt;
//     x_pred(2) = x_(2) + w * dt;
//     x_pred(3) = v;

//     Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
//     F(0,2) = -v * sin(theta) * dt;
//     F(0,3) = cos(theta) * dt;
//     F(1,2) =  v * cos(theta) * dt;
//     F(1,3) = sin(theta) * dt;
//     F(2,3) = tan(steering_angle) / wheelbase * dt;

//     Eigen::Matrix4d Q = Eigen::Matrix4d::Identity() * 0.01;

//     P_ = F * P_ * F.transpose() + Q;
//     x_ = x_pred;
//   }

//   void update(double measured_theta, double measured_yaw_rate, double measured_velocity,
//               double steering_angle, double wheelbase,
//               double R_theta, double R_w, double R_v) {
//     double theta_pred = x_(2);
//     double v = x_(3);
//     double w_pred = v * tan(steering_angle) / wheelbase;

//     Eigen::Vector3d z, h, y;
//     z << measured_theta, measured_yaw_rate, measured_velocity;
//     h << theta_pred, w_pred, v;
//     y = z - h;

//     while (y(0) > M_PI) y(0) -= 2 * M_PI;
//     while (y(0) < -M_PI) y(0) += 2 * M_PI;

//     Eigen::Matrix<double, 3, 4> H;
//     H << 0, 0, 1, 0,
//          0, 0, 0, tan(steering_angle) / wheelbase,
//          0, 0, 0, 1;

//     Eigen::Matrix3d R;
//     R << R_theta, 0, 0,
//          0, R_w, 0,
//          0, 0, R_v;

//     Eigen::Matrix3d S = H * P_ * H.transpose() + R;
//     Eigen::Matrix<double, 4, 3> K = P_ * H.transpose() * S.inverse();

//     x_ = x_ + K * y;
//     P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
//   }

//   Eigen::Vector4d getState() {
//     return x_;
//   }

// private:
//   Eigen::Vector4d x_;
//   Eigen::Matrix4d P_;
// };

// class VescToOdomWithEKF : public rclcpp::Node {
// public:
//   VescToOdomWithEKF(const rclcpp::NodeOptions & options)
//   : Node("vesc_to_odom_with_ekf_node", options),
//     x_(0.0), y_(0.0), initial_yaw_(std::numeric_limits<double>::quiet_NaN()),
//     imu_yaw_rate(0.0), real_yaw_rate(0.0), last_servo_cmd_(nullptr),
//     last_measured_velocity(0.0)
//   {
//     odom_frame_ = declare_parameter("odom_frame", "odom");
//     base_frame_ = declare_parameter("base_frame", "base_link");
//     speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 0.0);
//     speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
//     wheelbase_ = declare_parameter("wheelbase", 0.32);
//     steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", 0.0);
//     steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset", 0.0);

//     use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);

//     odom_pub_ = create_publisher<odom_msgs::msg::MyOdom>("my_odom", 10);
//     vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
//       "sensors/core", 10, std::bind(&VescToOdomWithEKF::vescStateCallback, this, _1));
//     imu_sub_ = create_subscription<vesc_msgs::msg::VescImuStamped>(
//       "sensors/imu", 10, std::bind(&VescToOdomWithEKF::imuCallback, this, _1));

//     if (use_servo_cmd_) {
//       servo_sub_ = create_subscription<std_msgs::msg::Float64>(
//          "sensors/servo_position_command", 10, std::bind(&VescToOdomWithEKF::servoCmdCallback, this, _1));
//     }

//     last_time_ = this->now();
//   }

// private:
//   void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state) {
//     double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
//     if (std::fabs(current_speed) < 0.05)
//       current_speed = 0.0;

//     last_measured_velocity = current_speed;

//     double current_steering_angle = 0.0;
//     if (use_servo_cmd_ && last_servo_cmd_ != nullptr) {
//       current_steering_angle = (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
//     }

//     rclcpp::Time current_time = state->header.stamp;
//     double dt = (current_time - last_time_).seconds();
//     last_time_ = current_time;

//     ekf_.predict(dt, current_steering_angle, wheelbase_);

//     Eigen::Vector4d state_est = ekf_.getState();
//     x_ = state_est(0);
//     y_ = state_est(1);
//     double theta_est = state_est(2);
//     double velocity = state_est(3);
    
//     double theta_est_deg = theta_est * 180 / M_PI;

//     double w_control = velocity * tan(current_steering_angle) / wheelbase_;

//     odom_msgs::msg::MyOdom odom_msg;
//     odom_msg.header.stamp = state->header.stamp;
//     odom_msg.header.frame_id = odom_frame_;
//     odom_msg.x = x_;
//     odom_msg.y = y_;
//     odom_msg.yaw = theta_est_deg;
//     odom_msg.linear_velocity = velocity;
//     odom_msg.angular_velocity = w_control;

//     odom_pub_->publish(odom_msg);
//   }

//   void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu) {
//     double measured_yaw_deg = -imu->imu.ypr.z;
//     double measured_yaw_rad = measured_yaw_deg * M_PI / 180.0;
    
//     imu_yaw_rate = -imu->imu.angular_velocity.z * M_PI / 180;
//     real_yaw_rate = imu_yaw_rate * (1 - 0.15 / 0.32);

//     if (std::isnan(initial_yaw_)) {
//       initial_yaw_ = measured_yaw_rad;
//       RCLCPP_INFO(this->get_logger(), "Initial Yaw Set: %f rad", initial_yaw_);
//       return;
//     }

//     double corrected_yaw = measured_yaw_rad - initial_yaw_;

//     while (corrected_yaw > M_PI) corrected_yaw -= 2 * M_PI;
//     while (corrected_yaw < -M_PI) corrected_yaw += 2 * M_PI;

//     double R_val = 0.05;
//     ekf_.update(corrected_yaw, real_yaw_rate, last_measured_velocity, 0.0, wheelbase_, R_val, 0.1, 0.05);
//   }

//   void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo) {
//     last_servo_cmd_ = servo;
//   }

//   rclcpp::Publisher<odom_msgs::msg::MyOdom>::SharedPtr odom_pub_;
//   rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
//   rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub_;
//   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

//   rclcpp::Time last_time_;
  
//   double x_, y_;
//   double initial_yaw_;
//   double imu_yaw_rate;
//   double real_yaw_rate;

//   std::string odom_frame_;
//   std::string base_frame_;
//   double speed_to_erpm_gain_;
//   double speed_to_erpm_offset_;
//   double wheelbase_;
//   double steering_to_servo_gain_;
//   double steering_to_servo_offset_;

//   bool use_servo_cmd_;
//   std::shared_ptr<std_msgs::msg::Float64> last_servo_cmd_;

//   double last_measured_velocity;

//   EKF ekf_;
// };

// } // namespace vesc_ackermann

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdomWithEKF)


#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "odom_msgs/msg/my_odom.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

namespace vesc_ackermann {

// Simple EKF class to estimate [x, y, theta]
class EKF {
public:
  EKF() {
    x_ = Eigen::Vector3d::Zero(); // [x, y, theta]
    P_ = Eigen::Matrix3d::Identity() * 0.1;
  }

  void predict(double dt, double v, double w) {
    double theta = x_(2);
    Eigen::Vector3d x_pred;
    x_pred(0) = x_(0) + v * cos(theta) * dt;
    x_pred(1) = x_(1) + v * sin(theta) * dt;
    x_pred(2) = x_(2) + w * dt;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * sin(theta) * dt;
    F(1,2) =  v * cos(theta) * dt;

    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 0.01;

    P_ = F * P_ * F.transpose() + Q;
    x_ = x_pred;
  }

  void update(double measured_theta, double R_val) {
    double theta_pred = x_(2);
    double y = measured_theta - theta_pred;

    while (y > M_PI) y -= 2 * M_PI;
    while (y < -M_PI) y += 2 * M_PI;

    Eigen::RowVector3d H;
    H << 0, 0, 1;

    double S = H * P_ * H.transpose() + R_val;
    Eigen::Vector3d K = P_ * H.transpose() / S;

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
  }

  Eigen::Vector3d getState() {
    return x_;
  }

private:
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
};

class VescToOdomWithEKF : public rclcpp::Node {
public:
  VescToOdomWithEKF(const rclcpp::NodeOptions & options)
  : Node("vesc_to_odom_with_ekf_node", options),
    x_(0.0), y_(0.0), initial_yaw_(std::numeric_limits<double>::quiet_NaN()),
    imu_yaw_rate(0.0), real_yaw_rate(0.0), last_servo_cmd_(nullptr)  
  {
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 0.0);
    speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
    wheelbase_ = declare_parameter("wheelbase", 0.33);
    steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", 0.0);
    steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset", 0.0);
    use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);
    imu_offset_ = declare_parameter("imu_y_offset", 0.15);

    odom_pub_ = create_publisher<odom_msgs::msg::MyOdom>("my_odom", 10);
    vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
      "sensors/core", 10, std::bind(&VescToOdomWithEKF::vescStateCallback, this, _1));
    imu_sub_ = create_subscription<vesc_msgs::msg::VescImuStamped>(
      "sensors/imu", 10, std::bind(&VescToOdomWithEKF::imuCallback, this, _1));

    if (use_servo_cmd_) {
      servo_sub_ = create_subscription<std_msgs::msg::Float64>(
         "sensors/servo_position_command", 10, std::bind(&VescToOdomWithEKF::servoCmdCallback, this, _1));
    }

    last_time_ = this->now();
  }

private:
  void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state) {
    double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(current_speed) < 0.05)
      current_speed = 0.0;

    double current_steering_angle = 0.0;
    if (use_servo_cmd_ && last_servo_cmd_ != nullptr) {
      current_steering_angle = (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    }
    double w_control = current_speed * tan(current_steering_angle) / wheelbase_;

    rclcpp::Time current_time = state->header.stamp;
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    ekf_.predict(dt, current_speed, real_yaw_rate);

    Eigen::Vector3d state_est = ekf_.getState();
    x_ = state_est(0);
    y_ = state_est(1);
    double theta_est = state_est(2);
    
    double theta_est_deg = theta_est * 180 / M_PI;

    odom_msgs::msg::MyOdom odom_msg;
    odom_msg.header.stamp = state->header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.x = x_;
    odom_msg.y = y_;
    odom_msg.yaw = theta_est_deg;
    odom_msg.linear_velocity = current_speed;
    odom_msg.angular_velocity = w_control;

    odom_pub_->publish(odom_msg);
  }

  void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu) {
    double measured_yaw_deg = -imu->imu.ypr.z;
    double measured_yaw_rad = measured_yaw_deg * M_PI / 180.0;
    
    imu_yaw_rate = -imu->imu.angular_velocity.z * M_PI / 180;
    real_yaw_rate = imu_yaw_rate * (1 - imu_offset_ / wheelbase_);

    if (std::isnan(initial_yaw_)) {
      initial_yaw_ = measured_yaw_rad;
      RCLCPP_INFO(this->get_logger(), "Initial Yaw Set: %f rad", initial_yaw_);
      return;
    }

    double corrected_yaw = measured_yaw_rad - initial_yaw_;

    while (corrected_yaw > M_PI) corrected_yaw -= 2 * M_PI;
    while (corrected_yaw < -M_PI) corrected_yaw += 2 * M_PI;

    double R_val = 0.05;
    ekf_.update(corrected_yaw, R_val);
  }

  void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo) {
    last_servo_cmd_ = servo;
  }

  rclcpp::Publisher<odom_msgs::msg::MyOdom>::SharedPtr odom_pub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

  rclcpp::Time last_time_;
  
  double x_, y_;
  double initial_yaw_;
  double imu_yaw_rate;
  double real_yaw_rate;

  std::string odom_frame_;
  std::string base_frame_;
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double wheelbase_;
  double steering_to_servo_gain_;
  double steering_to_servo_offset_;
  double imu_offset_;

  bool use_servo_cmd_;
  std::shared_ptr<std_msgs::msg::Float64> last_servo_cmd_;

  EKF ekf_;
};

} // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdomWithEKF)
