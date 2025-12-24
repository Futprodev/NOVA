#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

static inline double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a <= -M_PI) a += 2.0*M_PI;
  return a;
}

class PlanarIKHWNode : public rclcpp::Node {
public:
  PlanarIKHWNode() : Node("nova_arm_planar_ik_hw")
  {
    // link params (match MATLAB / URDF)
    d1_ = declare_parameter("d1", 0.145);
    L1_ = declare_parameter("L1", 0.200);
    L2_ = declare_parameter("L2", 0.255);
    L3_ = declare_parameter("L3", 0.0);

    // sign + offsets: math → hardware deg
    zero_offs_ = declare_parameter<std::vector<double>>(
        "zero_offsets", {0.0, 90.0, 0.0});      // degrees
    signs_ = declare_parameter<std::vector<double>>(
        "signs", {1.0, -1.0, 1.0});            // +1 / -1

    elbow_sign_pref_ = declare_parameter("elbow_sign", -1); // +1 or -1

    // hard limits in hardware deg (base, shoulder, elbow)
    lim_lower_deg_ = {-150.0, -94.0, -143.0};
    lim_upper_deg_ = {150.0, 104.0,  157.0};

    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "nova_arm/command_deg", 10);

    sub_goal_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "planar_goal", 10,
        std::bind(&PlanarIKHWNode::onGoal, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Planar IK HW node ready.");
  }

private:
  void onGoal(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto &d = msg->data;
    if (d.size() < 3) {
      RCLCPP_WARN(get_logger(), "planar_goal expects [x,y,z,(optional phi),(optional g)]");
      return;
    }

    const double x = d[0];
    const double y = d[1];
    const double z = d[2];

    // base yaw
    const double q1 = std::atan2(y, x);
    const double r  = std::sqrt(x*x + y*y);

    // project into sagittal plane
    const double Xs = r;
    const double Zs = z - d1_;

    // reach check
    const double D2 = Xs*Xs + Zs*Zs;
    const double D  = std::sqrt(D2);
    const double sumL = L1_ + L2_;
    const double diffL = std::fabs(L1_ - L2_);

    if (D > sumL + 1e-6 || D < diffL - 1e-6) {
      RCLCPP_WARN(get_logger(), "Target out of reach: D=%.3f, range=[%.3f, %.3f]", D, diffL, sumL);
      return;
    }

    // elbow angle
    double c3 = (D2 - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);
    c3 = std::max(-1.0, std::min(1.0, c3));
    double s_sq = std::max(0.0, 1.0 - c3*c3);
    double s3 = std::sqrt(s_sq);
    if (elbow_sign_pref_ < 0) s3 = -s3;
    const double q3 = std::atan2(s3, c3);

    // shoulder angle
    const double k1 = L1_ + L2_*std::cos(q3);
    const double k2 = L2_*std::sin(q3);
    const double q2 = std::atan2(Zs, Xs) - std::atan2(k2, k1);

    // math → hardware (deg)
    double q_math[3] = {wrap_pi(q1), wrap_pi(q2), wrap_pi(q3)};
    double q_deg[3];

    for (int i = 0; i < 3; ++i) {
      double base_deg = q_math[i] * 180.0 / M_PI;
      q_deg[i] = signs_[i] * base_deg + zero_offs_[i];
      q_deg[i] = std::max(lim_lower_deg_[i], std::min(q_deg[i], lim_upper_deg_[i]));
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = {q_deg[0], q_deg[1], q_deg[2]};
    pub_cmd_->publish(out);

    RCLCPP_INFO(get_logger(),
        "Cmd deg: [%.2f, %.2f, %.2f] for x=%.3f y=%.3f z=%.3f",
        q_deg[0], q_deg[1], q_deg[2], x, y, z);
  }

  // params
  double d1_, L1_, L2_, L3_;
  int elbow_sign_pref_;
  std::vector<double> zero_offs_, signs_;
  std::vector<double> lim_lower_deg_, lim_upper_deg_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarIKHWNode>());
  rclcpp::shutdown();
  return 0;
} 
