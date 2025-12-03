#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

static inline double wrap_pi(double a){while(a>M_PI)a-=2*M_PI;while(a<=-M_PI)a+=2*M_PI;return a;}
static inline double clamp(double v,double lo,double hi){return std::max(lo,std::min(v,hi));}

struct FKOut{double x,y,z;};

class PlanarIKRvizNode : public rclcpp::Node {
public:
  PlanarIKRvizNode(): Node("nova_arm_planar_ik_rviz")
  {
    zero_offs_ = declare_parameter<std::vector<double>>(
      "zero_offsets", {0.0, M_PI/2, M_PI});      // [j1, j2, j3] offsets
    signs_ = declare_parameter<std::vector<double>>(
      "signs", {1.0, -1.0, -1.0});               // [j1, j2, j3] direction flips
    // ---- Parameters ----
    d1_ = declare_parameter("d1", 0.151);   // base -> shoulder height
    L1_ = declare_parameter("L1", 0.200);   // shoulder -> elbow
    L2_ = declare_parameter("L2", 0.175);   // elbow -> wrist
    L3_ = declare_parameter("L3", 0.0);     // wrist -> EE offset along last link
    elbow_sign_pref_ = declare_parameter("elbow_sign", 1); // +1 or -1

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1","joint_2","joint_3","joint_4","joint_5"});

    // Limits (from URDF)
    lim_lower_ = { -M_PI,  -1.919, -2.356,  0.0,   -0.014 };
    lim_upper_ = {  M_PI,   1.919,  2.356,  0.014,  0.0   };

    vel_max_   = {1.0, 1.0, 1.5, 0.02, 0.02}; // rad/s or m/s

    current_q_ = std::vector<double>(5, 0.0);
    target_q_  = current_q_;
    start_q_   = current_q_;
    move_T_ = 0.5;
    move_start_ = now();

    pub_js_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);
    sub_goal_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "planar_goal", 10, std::bind(&PlanarIKRvizNode::onGoal, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
      std::bind(&PlanarIKRvizNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "IK node ready. d1=%.3f L1=%.3f L2=%.3f L3=%.3f (elbow_sign=%d)",
      d1_, L1_, L2_, L3_, elbow_sign_pref_);
  }

private:
  std::vector<double> zero_offs_;
  std::vector<double> signs_;
  
  // Forward kinematics for the 3 revolute joints (EE at the tip of L2+L3)
  FKOut fk(double q1,double q2,double q3,double L3_use=0.0) const {
    const double xp = L1_*std::cos(q2) + L2_*std::cos(q2+q3) + L3_use*std::cos(q2+q3);
    const double zp = d1_ + L1_*std::sin(q2) + L2_*std::sin(q2+q3) + L3_use*std::sin(q2+q3);
    const double c1 = std::cos(q1), s1 = std::sin(q1);
    return FKOut{ c1*xp, s1*xp, zp };
  }

  void onGoal(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto& d = msg->data;
    if (d.size() < 3) {
      RCLCPP_WARN(get_logger(), "planar_goal expects [x,y,z,(optional φ),(optional g)]. Got %zu", d.size());
      return;
    }

    // Inputs
    const double x = d[0], y = d[1], z = d[2];
    bool use_phi = (d.size() >= 4);
    double phi = use_phi ? d[3] : 0.0;
    const double g = (d.size()==3 ? 0.0 :
                     (d.size()==4 ? 0.0 : d[4]));

    // Base yaw
    const double q1 = std::atan2(y, x);
    const double r  = std::sqrt(x*x + y*y);

    RCLCPP_INFO(get_logger(),
      "Goal: x=%.3f y=%.3f z=%.3f %s φ=%.3f g=%.3f | q1(yaw)=%.2f° r=%.3f",
      x,y,z, use_phi?"with":"(no)", phi, g, (q1*180.0/M_PI), r);

    std::vector<std::vector<double>> cands;

    if (!use_phi) {
      // ---- Unconstrained IK (match MATLAB) ----
      const double Xs = r;
      const double Zs = z - d1_;
      const double D2 = Xs*Xs + Zs*Zs;
      const double D  = std::sqrt(D2);
      const double sumL = L1_ + L2_, diffL = std::fabs(L1_ - L2_);
      if (D > sumL + 1e-6 || D < diffL - 1e-6) {
        RCLCPP_WARN(get_logger(),
          "Target unreachable (unconstrained). D=%.3f, range=[%.3f, %.3f]", D, diffL, sumL);
        return;
      }
      double c3 = (D2 - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);
      c3 = std::max(-1.0, std::min(1.0, c3));
      const double s_sq = std::max(0.0, 1.0 - c3*c3);
      for (int s_first : {elbow_sign_pref_, -elbow_sign_pref_}) {
        const double s3 = (s_first>=0 ? +1.0 : -1.0) * std::sqrt(s_sq);
        const double q3 = std::atan2(s3, c3);
        const double k1 = L1_ + L2_*std::cos(q3);
        const double k2 = L2_*std::sin(q3);
        const double q2 = std::atan2(Zs, Xs) - std::atan2(k2, k1);
        std::vector<double> q = {wrap_pi(q1), wrap_pi(q2), wrap_pi(q3)};
        if (within_limits(q)) cands.push_back(q);
        if (!cands.empty()) break; // prefer requested branch
      }
    } else {
      // ---- Constrained IK (old behavior; honor φ) ----
      const double rw = r - L3_*std::cos(phi);
      const double zw = (z - d1_) - L3_*std::sin(phi);
      const double D2 = rw*rw + zw*zw;
      const double D  = std::sqrt(D2);
      const double sumL = L1_ + L2_, diffL = std::fabs(L1_ - L2_);
      RCLCPP_INFO(get_logger(), "Constrained: rw=%.3f zw=%.3f D=%.3f", rw, zw, D);
      if (D > sumL + 1e-6 || D < diffL - 1e-6) {
        RCLCPP_WARN(get_logger(),
          "Target unreachable (constrained). D=%.3f, range=[%.3f, %.3f]", D, diffL, sumL);
        return;
      }
      double c2 = (D2 - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);
      c2 = std::max(-1.0, std::min(1.0, c2));
      const double s_sq = std::max(0.0, 1.0 - c2*c2);
      for (int s_first : {elbow_sign_pref_, -elbow_sign_pref_}) {
        const double s2 = (s_first>=0 ? +1.0 : -1.0) * std::sqrt(s_sq);
        const double q3 = std::atan2(s2, c2);
        const double k1 = L1_ + L2_*std::cos(q3);
        const double k2 = L2_*std::sin(q3);
        const double q2 = std::atan2(zw, rw) - std::atan2(k2, k1);
        std::vector<double> q = {wrap_pi(q1), wrap_pi(q2), wrap_pi(q3)};
        if (within_limits(q)) cands.push_back(q);
        if (!cands.empty()) break;
      }
    }

    if (cands.empty()) {
      RCLCPP_WARN(get_logger(), "IK found no solution within joint limits.");
      return;
    }

    // Choose candidate and set gripper
    const auto q = cands.front();
    const double q4 = clamp(+0.5*g, lim_lower_[3], lim_upper_[3]);
    const double q5 = clamp(-0.5*g, lim_lower_[4], lim_upper_[4]);
    target_q_ = {q[0], q[1], q[2], q4, q5};

    // Debug: print angles and verify with FK
    const FKOut ee = fk(q[0], q[1], q[2], L3_);
    RCLCPP_INFO(get_logger(),
      "Math q=[%.2f, %.2f, %.2f] deg  FK=(%.3f, %.3f, %.3f)",
      q[0]*180/M_PI, q[1]*180/M_PI, q[2]*180/M_PI, ee.x, ee.y, ee.z);

    // What RViz will actually receive:
    const double q1_cmd = signs_[0]*q[0] + zero_offs_[0];
    const double q2_cmd = signs_[1]*q[1] + zero_offs_[1];
    const double q3_cmd = signs_[2]*q[2] + zero_offs_[2];
    RCLCPP_INFO(get_logger(),
      "Cmd q (to RViz)=[%.2f, %.2f, %.2f] deg",
      q1_cmd*180/M_PI, q2_cmd*180/M_PI, q3_cmd*180/M_PI);

    // Time-scaling
    double t_min = 0.0;
    for (size_t i=0;i<target_q_.size();++i){
      const double dq = std::fabs(target_q_[i]-current_q_[i]);
      const double t_i = dq/std::max(1e-6, vel_max_[i]);
      if (t_i>t_min) t_min=t_i;
    }
    move_T_ = std::max(0.3, t_min);
    start_q_ = current_q_;
    move_start_ = now();
  }

  void onTimer(){
    const rclcpp::Time t = now();
    const double alpha = std::min(1.0,(t-move_start_).seconds()/std::max(1e-6,move_T_));

    for(size_t i=0;i<current_q_.size();++i)
      current_q_[i] = start_q_[i] + alpha*(target_q_[i]-start_q_[i]);

    sensor_msgs::msg::JointState js;
    js.header.stamp = t;
    js.name = joint_names_;
    js.position.resize(current_q_.size());

    // math → display
    js.position[0] = signs_[0]*current_q_[0] + zero_offs_[0];   // joint_1
    js.position[1] = signs_[1]*current_q_[1] + zero_offs_[1];   // joint_2 (−q + 90°)
    js.position[2] = signs_[2]*current_q_[2] + zero_offs_[2];   // joint_3 (−q + 180°)
    js.position[3] = current_q_[3];                             // gripper L (unchanged)
    js.position[4] = current_q_[4];                             // gripper R (unchanged)

    pub_js_->publish(js);
  }


  bool within_limits(const std::vector<double>& q){
    for(int i=0;i<3;++i)
      if(q[i] < lim_lower_[i]-1e-9 || q[i] > lim_upper_[i]+1e-9) return false;
    return true;
  }

  // params
  double d1_, L1_, L2_, L3_;
  int elbow_sign_pref_;
  std::vector<std::string> joint_names_;
  std::vector<double> lim_lower_, lim_upper_, vel_max_;
  // state
  std::vector<double> current_q_, target_q_, start_q_;
  double move_T_; rclcpp::Time move_start_;
  // ROS I/O
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PlanarIKRvizNode>());
  rclcpp::shutdown();
  return 0;
}
