#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

static inline double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

static inline double rad2deg(double r){ return r * 180.0 / M_PI; }
static inline double deg2rad(double d){ return d * M_PI / 180.0; }

struct FKOut { double x, y, z; };

class PlanarIKRvizNode : public rclcpp::Node {
public:
  PlanarIKRvizNode() : Node("nova_arm_planar_ik_rviz")
  {
    // --- Parameters: match MATLAB kinematics ---
    d1_ = declare_parameter("d1", 0.151);
    L1_ = declare_parameter("L1", 0.200);
    L2_ = declare_parameter("L2", 0.175);
    L3_ = declare_parameter("L3", 0.0);

    // Same as MATLAB: elbowSign = -1
    elbow_sign_pref_ = declare_parameter("elbow_sign", -1);

    // === IMPORTANT: These are MATLAB cmd_hw mapping parameters IN DEGREES ===
    // cmd_hw_deg = signs_deg[i] * q_math_deg + zero_offs_deg[i]
    // MATLAB mapping: BASE=q1, SH=-q2+90, EL=q3  --> signs=[1,-1,1], zero=[0,90,0]
    zero_offs_deg_ = declare_parameter<std::vector<double>>(
      "zero_offsets_deg", {0.0, 90.0, 0.0});
    signs_deg_ = declare_parameter<std::vector<double>>(
      "signs_deg", {1.0, -1.0, -1.0});

    // MATLAB cmd_hw clamps (deg)
    cmd_lower_deg_ = declare_parameter<std::vector<double>>(
      "cmd_lower_deg", {-150.0, -94.0, -143.0});
    cmd_upper_deg_ = declare_parameter<std::vector<double>>(
      "cmd_upper_deg", { 150.0, 104.0,  157.0});

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"base_joint", "shoulder_joint", "elbow_joint"});

    // wide math limits (radians) - still fine
    lim_lower_ = declare_parameter<std::vector<double>>(
      "joint_lower", {-M_PI, -M_PI, -M_PI});
    lim_upper_ = declare_parameter<std::vector<double>>(
      "joint_upper", { M_PI,  M_PI,  M_PI});

    vel_max_ = declare_parameter<std::vector<double>>(
      "joint_vel_max", {1.5, 1.5, 1.5});  // rad/s

    current_q_ = std::vector<double>(3, 0.0);
    target_q_  = current_q_;
    start_q_   = current_q_;
    move_T_    = 0.5;
    move_start_ = now();

    pub_js_ = create_publisher<sensor_msgs::msg::JointState>("arm_joint_states", 50);

    sub_goal_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "planar_goal", 10,
      std::bind(&PlanarIKRvizNode::onGoal, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&PlanarIKRvizNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "RViz IK ready. d1=%.3f L1=%.3f L2=%.3f elbow_sign=%d (cmd mapping uses *_deg + clamps)",
      d1_, L1_, L2_, elbow_sign_pref_);
  }

private:
  FKOut fk(double q1, double q2, double q3) const {
    const double xp = L1_ * std::cos(q2) + L2_ * std::cos(q2 + q3);
    const double zp = d1_ + L1_ * std::sin(q2) + L2_ * std::sin(q2 + q3);
    const double c1 = std::cos(q1), s1 = std::sin(q1);
    return FKOut{ c1 * xp, s1 * xp, zp };
  }

  // map math(rad) -> cmd_hw(deg) + clamp
  std::array<double,3> mathToCmdClampedDeg(const std::array<double,3>& q_math_rad,
                                           std::array<double,3>* out_raw_deg=nullptr) const
  {
    std::array<double,3> raw{}, cmd{};
    for(int i=0;i<3;i++){
      const double q_deg = rad2deg(q_math_rad[i]);
      raw[i] = signs_deg_[i] * q_deg + zero_offs_deg_[i];
      cmd[i] = clamp(raw[i], cmd_lower_deg_[i], cmd_upper_deg_[i]);
    }
    if(out_raw_deg) *out_raw_deg = raw;
    return cmd;
  }

  // cmd_hw(deg) -> math(rad)  (inverse of the mapping)
  std::array<double,3> cmdDegToMathRad(const std::array<double,3>& cmd_deg) const
  {
    std::array<double,3> q{};
    for(int i=0;i<3;i++){
      const double s = signs_deg_[i];
      const double q_deg = (std::fabs(s) < 1e-9) ? 0.0 : (cmd_deg[i] - zero_offs_deg_[i]) / s;
      q[i] = wrap_pi(deg2rad(q_deg));
    }
    return q;
  }

  void onGoal(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto &d = msg->data;
    if (d.size() < 3) {
      RCLCPP_WARN(get_logger(), "planar_goal expects [x,y,z]. Got %zu", d.size());
      return;
    }

    const double x = d[0], y = d[1], z = d[2];

    const double q1 = std::atan2(y, x);
    const double r_xy = std::hypot(x, y);
    const double z_rel = z - d1_;
    const double r = std::hypot(r_xy, z_rel);

    const double sumL  = L1_ + L2_;
    const double diffL = std::fabs(L1_ - L2_);

    if (r > sumL + 1e-6 || r < diffL - 1e-6) {
      RCLCPP_WARN(get_logger(), "Target unreachable. D=%.3f, range=[%.3f, %.3f]", r, diffL, sumL);
      return;
    }

    double cos_q3 = (r*r - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);
    cos_q3 = clamp(cos_q3, -1.0, 1.0);

    double q3 = elbow_sign_pref_ * std::acos(cos_q3);

    const double beta  = std::atan2(z_rel, r_xy);
    const double gamma = std::atan2(L2_ * std::sin(q3),
                                    L1_ + L2_ * std::cos(q3));
    double q2 = beta - gamma;

    std::array<double,3> q_math = {wrap_pi(q1), wrap_pi(q2), wrap_pi(q3)};

    // === MATLAB cmd_hw mapping + clamp ===
    std::array<double,3> cmd_raw_deg{}, cmd_deg{};
    cmd_deg = mathToCmdClampedDeg(q_math, &cmd_raw_deg);

    // Clamp the target IN COMMAND SPACE, then invert back to math space.
    // This makes RViz settle at the clamped pose (like hardware).
    std::array<double,3> q_math_clamped = cmdDegToMathRad(cmd_deg);

    std::vector<double> q = {q_math_clamped[0], q_math_clamped[1], q_math_clamped[2]};
    if (!within_limits(q)) {
      RCLCPP_WARN(get_logger(), "Clamped IK violates math joint limits.");
      return;
    }

    target_q_ = q;

    // Debug: show both math IK and cmd_hw (raw+clamped)
    {
      const FKOut ee = fk(q_math[0], q_math[1], q_math[2]);
      RCLCPP_INFO(get_logger(),
        "MathIK q=[%.2f, %.2f, %.2f] deg  FK=(%.3f, %.3f, %.3f)",
        rad2deg(q_math[0]), rad2deg(q_math[1]), rad2deg(q_math[2]),
        ee.x, ee.y, ee.z);

      RCLCPP_INFO(get_logger(),
        "cmd_hw raw=[%.2f %.2f %.2f]  clamped=[%.2f %.2f %.2f] deg",
        cmd_raw_deg[0], cmd_raw_deg[1], cmd_raw_deg[2],
        cmd_deg[0],     cmd_deg[1],     cmd_deg[2]);
    }

    // timing
    double t_min = 0.0;
    for (size_t i = 0; i < target_q_.size(); ++i) {
      const double dq = std::fabs(target_q_[i] - current_q_[i]);
      const double t_i = dq / std::max(1e-6, vel_max_[i]);
      t_min = std::max(t_min, t_i);
    }

    move_T_ = std::max(0.3, t_min);
    start_q_ = current_q_;
    move_start_ = now();
  }

  void onTimer()
  {
    const rclcpp::Time t = now();
    const double alpha = std::min(1.0, (t - move_start_).seconds() / std::max(1e-6, move_T_));

    for (size_t i = 0; i < current_q_.size(); ++i) {
      current_q_[i] = start_q_[i] + alpha * (target_q_[i] - start_q_[i]);
    }

    // Publish JointState in radians, but based on MATLAB cmd_hw(deg) mapping+clamp
    std::array<double,3> q_math = {current_q_[0], current_q_[1], current_q_[2]};
    std::array<double,3> cmd_deg = mathToCmdClampedDeg(q_math);

    sensor_msgs::msg::JointState js;
    js.header.stamp = t;
    js.name = joint_names_;
    js.position.resize(3);

    for (int i = 0; i < 3; ++i) {
      js.position[i] = deg2rad(cmd_deg[i]);  // RViz expects rad
    }

    pub_js_->publish(js);
  }

  bool within_limits(const std::vector<double> &q) {
    for (int i = 0; i < 3; ++i) {
      if (q[i] < lim_lower_[i] - 1e-9 || q[i] > lim_upper_[i] + 1e-9)
        return false;
    }
    return true;
  }

  // params
  double d1_, L1_, L2_, L3_;
  int elbow_sign_pref_;
  std::vector<std::string> joint_names_;
  std::vector<double> lim_lower_, lim_upper_, vel_max_;

  // MATLAB cmd_hw mapping + clamp (DEGREES)
  std::vector<double> zero_offs_deg_, signs_deg_;
  std::vector<double> cmd_lower_deg_, cmd_upper_deg_;

  // state (math radians)
  std::vector<double> current_q_, target_q_, start_q_;
  double move_T_;
  rclcpp::Time move_start_;

  // ROS I/O
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarIKRvizNode>());
  rclcpp::shutdown();
  return 0;
}
