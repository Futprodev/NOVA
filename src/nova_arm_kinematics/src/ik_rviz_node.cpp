#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

static inline double wrap_pi(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a <= -M_PI) a += 2*M_PI;
  return a;
}
static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

class PlanarIKRvizNode : public rclcpp::Node {
public:
  PlanarIKRvizNode() : Node("nova_arm_planar_ik_rviz")
  {
    // --- Planar XY, all revolute about +Z (matches URDF) ---
    L1_ = declare_parameter("L1", 0.066);       // base->joint_2 projection (m)
    L2_ = declare_parameter("L2", 0.200);       // joint_2->joint_3 (m)
    L3_ = declare_parameter("L3", 0.17556);     // joint_3->EE center (m)

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1","joint_2","joint_3","joint_4","joint_5"});

    // Limits per latest URDF
    lim_lower_ = {-M_PI, -M_PI/2.0, -2.332,  0.0,   -0.014};
    lim_upper_ = { M_PI,  M_PI/2.0,  2.332,  0.014,  0.0};

    // Max "speeds" for interpolation (RViz only)
    vel_max_   = {1.0, 1.0, 1.5, 0.02, 0.02};  // rad/s or m/s

    current_q_ = std::vector<double>(5, 0.0);
    target_q_  = current_q_;
    start_q_   = current_q_;
    move_T_ = 0.5;
    move_start_ = now();

    pub_js_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);

    sub_goal_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "planar_goal", 10, std::bind(&PlanarIKRvizNode::onGoal, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(10), // 100 Hz
      std::bind(&PlanarIKRvizNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "RViz IK node ready (XY). L1=%.3f L2=%.3f L3=%.5f", L1_, L2_, L3_);
  }

private:
  void onGoal(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(), "planar_goal expects [x, y, phi, g_open]");
      return;
    }
    const double x   = msg->data[0];
    const double y   = msg->data[1];
    const double phi = msg->data[2];    // radians
    const double g   = msg->data[3];    // meters (total opening)

    // Wrist center in XY after removing L3 along EE orientation
    const double wx = x - L3_*std::cos(phi);
    const double wy = y - L3_*std::sin(phi);

    const double r2 = wx*wx + wy*wy;
    const double c2 = (r2 - L1_*L1_ - L2_*L2_) / (2.0*L1_*L2_);
    if (std::abs(c2) > 1.0) {
      RCLCPP_WARN(get_logger(), "Target unreachable (|c2|=%.3f).", c2);
      return;
    }

    // Try elbow-up then elbow-down
    std::vector<std::vector<double>> cands;
    const double s_sq = std::max(0.0, 1.0 - c2*c2);
    for (int s : {+1, -1}) {
      const double q2 = std::atan2(s*std::sqrt(s_sq), c2);
      const double q1 = std::atan2(wy, wx) - std::atan2(L2_*std::sin(q2), L1_ + L2_*std::cos(q2));
      const double q3 = phi - q1 - q2;

      std::vector<double> q = { wrap_pi(q1), wrap_pi(q2), wrap_pi(q3) };
      if (within_limits(q)) cands.push_back(q);
    }
    if (cands.empty()) {
      RCLCPP_WARN(get_logger(), "IK found no solution within joint limits.");
      return;
    }

    // Choose first candidate (closest-to-current selection could be added)
    std::vector<double> q = cands.front();

    // Symmetric jaws
    const double q4 = clamp(+0.5*g, lim_lower_[3], lim_upper_[3]);
    const double q5 = clamp(-0.5*g, lim_lower_[4], lim_upper_[4]);

    target_q_ = { q[0], q[1], q[2], q4, q5 };

    // Time-scaling based on per-joint limits
    double t_min = 0.0;
    for (size_t i=0;i<target_q_.size();++i) {
      const double dq = std::abs(target_q_[i] - current_q_[i]);
      const double t_i = dq / std::max(1e-6, vel_max_[i]);
      if (t_i > t_min) t_min = t_i;
    }
    move_T_ = std::max(0.3, t_min);
    start_q_ = current_q_;
    move_start_ = now();
  }

  void onTimer() {
    const rclcpp::Time t = now();
    const double alpha = std::min(1.0, (t - move_start_).seconds() / std::max(1e-6, move_T_));
    for (size_t i=0;i<current_q_.size();++i) {
      current_q_[i] = start_q_[i] + alpha * (target_q_[i] - start_q_[i]);
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = t;
    js.name = joint_names_;
    js.position = current_q_;
    pub_js_->publish(js);
  }

  bool within_limits(const std::vector<double>& q) {
    for (int i=0;i<3;i++) {
      if (q[i] < lim_lower_[i] - 1e-9 || q[i] > lim_upper_[i] + 1e-9) return false;
    }
    return true;
  }

  // params
  double L1_, L2_, L3_;
  std::vector<std::string> joint_names_;
  std::vector<double> lim_lower_, lim_upper_, vel_max_;

  // state
  std::vector<double> current_q_, target_q_, start_q_;
  double move_T_;
  rclcpp::Time move_start_;

  // ROS I/O
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarIKRvizNode>());
  rclcpp::shutdown();
  return 0;
}
