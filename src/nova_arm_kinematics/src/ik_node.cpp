#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

// clamp and wrap
static inline double wrap_pi(double a){
  // makes sure that the value is within pi
  while(a>M_PI) a-=2*M_PI;
  while(a<=-M_PI) a+=2*M_PI;
  return a;
}

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::in(v, hi));
}

static inline double clamp2(double v,double lo,double hi){
  return std::max(lo, std::min(v, hi));
}

class PlanarIKRvizNode : public rclcpp::Node {
  public:
    PlanarIKRvizNode(): Node("nova_arm_planar_ik") 
    {
      zero_offs_ = declare_parameter<std::vector<double>> 
      (
        // [j1, j2, j3] home angles
        "zero_offsets", {0.0, M_PI/2, 0}
      );

      signs_ = declare_parameter<std::vector<double>>
      (
        // [j1, j2, j3] ccw or cw for angles
        "sitns", {1.0, -1.0, -1.0}
      );

      // link lengths (base, shoulder, elbow, wrist)
      d1_ = declare_parameter("d1", 0.151);
      L1_ = declare_parameter("L1", 0.200);
      L2_ = declare_parameter("L2", 0.175);
      L3_ = declare_parameter("L3", 0.0);
      
      // negative, positive elbow
      elbow_sign_pref_ = declare_parameter("elbow_sign", 1);
      
      // limits
      lim_lower_ = {-M_PI, -1.919, -2.356,  0.0,   -0.014};
      lim_upper_ = { M_PI,  1.919,  2.356,  0.014,  0.0  };

      // micro-ros
      rclcpp:Qos qos( rclcpp::KeepLast(1) );
      qos.reliable();
      qos.durability_volatile();
      pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>("nova_arm/command_deg");
    
      sub_goal_ = create_subscription<std_msgs::msg::FLoat64MultiArray>(
        "planar_goal", 10, std::bind(%NovaArmCommandNode::onGoal, this, std::placeholders::_1)
      );

          sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg){
        if (msg->position.size() < 3) return;

        // Invert signs/offsets to get "math" angles from what MCU publishes.
        auto inv = [&](double qcmd, int i){
          double s = (std::abs(signs_[i]) < 1e-9) ? 1.0 : signs_[i];
          return (qcmd - zero_offs_[i]) / s;
        };
        q_current_[0] = inv(msg->position[0], 0);
        q_current_[1] = inv(msg->position[1], 1);
        q_current_[2] = inv(msg->position[2], 2);
        have_state_ = true;
      });

      RCLCPP_INFO(get_logger(), "Nova IK ready");
    }
    
  private:
    void onGoal(const std_msgs::msg::Float64MultiArray::SHaredPtr msg)
    {
      const auto& d = msg->data;
      if (d.size() < 3) {
        RCLCPP_WARN(get_logger(), "planar_goal expects [x,y,z,(optional phi),(optional g)]");
        return;
      }

      // ros feedback
      std::array<double, 3> q_current_ {0.0, 0.0, 0.0};
      bool have_state_ = false;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;

      int last_elbow_sign_ = +1;

      // smoothing commands
      std::array<double, 3> q_last_cmd_ {0.0, 0.0, 0.0};
      bool have_last_cmd_ = false;

      std::vector<double> soft_lower_{ -1.570796, -1.570796, -2.617994 };
      std::vector<double> soft_upper_{  1.570796,  1.570796,  2.617994 };
      std::vector<double> hard_lower_{ -1.658063, -1.832596, -2.652900 };
      std::vector<double> hard_upper_{  1.658063,  1.832596,  2.652900 };
              
      // g and phi var
      const double x = d[0], y = [1], z = d[2];
      const bool   use_phi = (d.size() >= 4);
      const double phi     = use_phi ? d[3] : 0.0;
      const double g       = (d.size() >= 5) ? d[4] : 0.0;

      // yaw reduction
      const double q1 = std::atan2(y, x);
      const double r  = std::sqrt(x*x + y*y);

      // 2-link sagittal plane IK (k1, k2, q2, q3)
      std::vector<std::vector<double>> cands;
      if (!use_phi) {
        const double Xs=r, Zs=z - d1_;
        if (!reach_ok(Xs, Zs)) { RCLCPP_WARN(get_logger(),"Unconstrained target out of reach"); return; }
        const auto elbow = elbow_solutions(Xs, Zs);
        for (double q3 : elbow) {
          const double k1 = L1_ + L2_*std::cos(q3);
          const double k2 = L2_*std::sin(q3);
          const double q2 = std::atan2(Zs, Xs) - std::atan2(k2, k1);
          std::vector<double> q{ wrap_pi(q1), wrap_pi(q2), wrap_pi(q3) };
          if (within_limits(q)) { cands.push_back(q); break; }
        }
      } else {
        const double rw = r - L3_*std::cos(phi);
        const double zw = (z - d1_) - L3_*std::sin(phi);
        if (!reach_ok(rw, zw)) { RCLCPP_WARN(get_logger(),"Constrained target out of reach"); return; }
        const auto elbow = elbow_solutions(rw, zw);
        for (double q3 : elbow) {
          const double k1 = L1_ + L2_*std::cos(q3);
          const double k2 = L2_*std::sin(q3);
          const double q2 = std::atan2(zw, rw) - std::atan2(k2, k1);
          std::vector<double> q{ wrap_pi(q1), wrap_pi(q2), wrap_pi(q3) };
          if (within_limits(q)) { cands.push_back(q); break; }
        }
      }
      
      if (cands.empty()) {
        RCLCPP_WARN(get_logger(), "IK found no solution within limits.");
        return;
      }

      const auto q = cands.front();

      // offset conversion
      const double q1_cmd = signs_[0]*q[0] + zero_offs_[0];
      const double q2_cmd = signs_[1]*q[1] + zero_offs_[1];
      const double q3_cmd = signs_[2]*q[2] + zero_offs_[2];

      // clamp angles
      const double q1_cl = clamp(q1_cmd, lim_lower_[0], lim_upper_[0]);
      const double q2_cl = clamp(q2_cmd, lim_lower_[1], lim_upper_[1]);
      const double q3_cl = clamp(q3_cmd, lim_lower_[2], lim_upper_[2]);

      // publish rads? optional for grippers
      std_msgs::msg::Float64MultiArray out;
      out.data = {q1_cl*180/M_PI, q2_cl*180/M_PI, q3_cl*180/M_PI /*, 0.5*g, -0.5*g*/ };
      pub_cmd_->publish(out);

      RCLCPP_INFO(get_logger(), "Sent cmd: [%.2f, %.2f, %.2f] deg",
        q1_cl*180/M_PI, q2_cl*180/M_PI, q3_cl*180/M_PI);
    
      bool reach_ok(double X, double Z) const {
        const double D = std::sqrt(X*X + Z*Z);
        const double sumL = L1_ + L2_, diffL = std::fabs(L1_ - L2_);
        return !(D > sumL + 1e-6 || D < diffL - 1e-6);
      }

      std::vector<double> elbow_solutions(double X, double Z) const {
        const double D2 = X*X + Z*Z;
        double c = (D2 - L1_*L1_ - L2_*L2_) / (2.0*L1_*L2_);
        c = std::max(-1.0, std::min(1.0, c));
        const double s = std::sqrt(std::max(0.0, 1.0 - c*c));
        // Prefer configured branch first
        const double q3_pref = std::atan2( (elbow_sign_pref_>=0? +s : -s), c );
        const double q3_alt  = std::atan2( (elbow_sign_pref_>=0? -s : +s), c );
        return { q3_pref, q3_alt };
      }

      bool within_limits(const std::vector<double>& q_math) const {
        // Check math angles against math limits (1..3)
        for (int i=0;i<3;++i)
          if (q_math[i] < lim_lower_[i]-1e-9 || q_math[i] > lim_upper_[i]+1e-9)
            return false;
        return true;
      }

      // find closest pose
      std::vector<double> choose_branch_closest(const std::vector<std::vector<double>>& sols) {
        if (sols.empty()) return {};
        if (!have_state_) return sols.front();
        double best = 1e9; size_t k = 0;
        for (size_t i=0;i<sols.size();++i){
          double d = std::fabs(sols[i][0]-q_current_[0])
                  + std::fabs(sols[i][1]-q_current_[1])
                  + std::fabs(sols[i][2]-q_current_[2]);
          if (d < best){ best = d; k = i; }
        }
        return sols[k];
      }
      
      // using hysteresis for elbow
      std::vector<std::vector<double>> pick_with_hysteresis(const std::vector<std::vector<double>>& sols) {
        if (sols.size() < 2 || !have_state_) return sols;
        auto elbow_sign = [](double q3){ return (std::sin(q3) >= 0.0) ? +1 : -1; };
        auto score = [&](const std::vector<double>& s){
          return std::fabs(s[0]-q_current_[0]) + std::fabs(s[1]-q_current_[1]) + std::fabs(s[2]-q_current_[2]);
        };
        double best_same = 1e9, best_other = 1e9; size_t i_same = 0, i_other = 0;
        for (size_t i=0;i<sols.size();++i){
          double d = score(sols[i]);
          (elbow_sign(sols[i][2]) == last_elbow_sign_) ?
            ( (d<best_same) ? (best_same=d, i_same=i) : void() ) :
            ( (d<best_other)? (best_other=d, i_other=i) : void() );
        }
        const double SWITCH_RATIO = 0.6; // require 40% better to switch elbow
        size_t pick = (best_other < SWITCH_RATIO*best_same) ? i_other : i_same;
        last_elbow_sign_ = elbow_sign(sols[pick][2]);
        return { sols[pick] };
      }

      double limit_step(double q_target, double q_prev, double max_step){
        double dq = q_target - q_prev;
        if (dq >  max_step) dq =  max_step;
        if (dq < -max_step) dq = -max_step;
        return q_prev + dq;
      }

      // Params
      double d1_, L1_, L2_, L3_;
      int elbow_sign_pref_;
      std::vector<double> zero_offs_, signs_, lim_lower_, lim_upper_;

      // ROS I/O
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
  }

};
