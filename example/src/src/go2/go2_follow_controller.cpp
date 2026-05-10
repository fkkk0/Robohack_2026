#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class Go2FollowController : public rclcpp::Node {
 public:
  Go2FollowController()
      : Node("go2_follow_controller"), sport_client_(this) {
    target_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "/follow/target", 10,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          // Reset the controller-side EMA when we just came back from a loss
          // so we don't blend stale smoothed values with a fresh target.
          const bool was_stale =
              !have_target_ || (now() - last_target_).seconds() > kTargetTimeout;
          if (was_stale) {
            filt_bearing_ = msg->x;
            filt_distance_ = msg->y;
          } else {
            filt_bearing_ =
                kTargetEmaAlpha * msg->x + (1.f - kTargetEmaAlpha) * filt_bearing_;
            filt_distance_ =
                kTargetEmaAlpha * msg->y + (1.f - kTargetEmaAlpha) * filt_distance_;
          }
          last_target_ = now();
          have_target_ = true;
        });

    state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        "lf/sportmodestate", 1,
        [this](unitree_go::msg::SportModeState::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          sport_mode_ = msg->mode;
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(kTickDt), [this] { Tick(); });

    // Send one final StopMove when the context is shutting down (Ctrl-C etc.).
    // rclcpp::on_shutdown fires before the context is torn down, so the
    // publisher inside SportClient is still valid here.
    rclcpp::on_shutdown([this]() { EmitStop(); });
  }

  // Callable from main() / on_shutdown to halt the robot on exit.
  void EmitStop() { sport_client_.StopMove(req_); }

 private:
  // Unsafe sport-mode values where Move() would fight the robot's own state
  // machine: 5 lieDown, 6 jointLock, 7 damping, 8 recoveryStand, 10 sit.
  static bool IsModeUnsafe(uint8_t mode) {
    return mode == 5 || mode == 6 || mode == 7 || mode == 8 || mode == 10;
  }

  static float SlewLimit(float prev, float target, float max_delta) {
    return std::clamp(target, prev - max_delta, prev + max_delta);
  }

  void Tick() {
    float bearing, distance;
    rclcpp::Time stamp;
    bool have;
    uint8_t mode;
    {
      std::lock_guard<std::mutex> lk(mu_);
      bearing = filt_bearing_;
      distance = filt_distance_;
      stamp = last_target_;
      have = have_target_;
      mode = sport_mode_;
    }

    if (IsModeUnsafe(mode)) {  // lieDown/jointLock/damping/recoveryStand/sit
      Stop();
      return;
    }

    if (!have || (now() - stamp).seconds() > kTargetTimeout) {
      Stop();
      return;
    }

    float dist_err = distance - kDesiredDistance;
    float vx = 0.f;
    if (std::fabs(dist_err) > kDistDeadband) {
      vx = std::clamp(kKpDist * dist_err, kVxMin, kVxMax);
    }
    if (distance < kMinSafeDist && vx > 0.f) vx = 0.f;

    float vyaw = 0.f;
    if (std::fabs(bearing) > kBearingDeadband) {
      vyaw = std::clamp(-kKpYaw * bearing, -kVyawMax, kVyawMax);
    }

    // When the target is far off-axis, throttle forward motion so we yaw
    // toward them first rather than driving a curve.
    if (std::fabs(bearing) > 0.4f) vx = std::min(vx, 0.1f);

    // Slew-limit the commanded velocities so the gait never jerks.
    vx = SlewLimit(prev_vx_, vx, kVxAccel * kTickDt);
    vyaw = SlewLimit(prev_vyaw_, vyaw, kVyawAccel * kTickDt);

    // Snap sub-threshold commands to zero so the dog doesn't "tick" in place.
    if (std::fabs(vx) < kVxDeadband) vx = 0.f;
    if (std::fabs(vyaw) < kVyawDeadband) vyaw = 0.f;

    prev_vx_ = vx;
    prev_vyaw_ = vyaw;

    sport_client_.Move(req_, vx, 0.f, vyaw);
  }

  // Stop cleanly and reset slew state so a resume starts from zero.
  void Stop() {
    sport_client_.StopMove(req_);
    prev_vx_ = 0.f;
    prev_vyaw_ = 0.f;
  }

  static constexpr double kTickDt = 0.05;  // 20 Hz control loop
  static constexpr float kDesiredDistance = 1.5f;
  static constexpr float kDistDeadband = 0.15f;
  static constexpr float kBearingDeadband = 0.05f;
  static constexpr float kKpDist = 0.8f;
  static constexpr float kKpYaw = 2.5f;
  static constexpr float kVxMax = 0.8f;
  static constexpr float kVxMin = -0.3f;
  static constexpr float kVyawMax = 1.2f;
  static constexpr float kMinSafeDist = 0.6f;
  static constexpr double kTargetTimeout = 0.5;
  // Max acceleration on commanded velocities (m/s² and rad/s²).
  static constexpr float kVxAccel = 1.5f;
  static constexpr float kVyawAccel = 3.0f;
  // Output deadband — suppress tiny commands that the gait can't execute cleanly.
  static constexpr float kVxDeadband = 0.08f;
  static constexpr float kVyawDeadband = 0.10f;
  // EMA on incoming targets (perception already smooths, so keep this light).
  static constexpr float kTargetEmaAlpha = 0.4f;

  SportClient sport_client_;
  unitree_api::msg::Request req_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mu_;
  // Smoothed target values fed to the controller.
  float filt_bearing_{0.f};
  float filt_distance_{0.f};
  rclcpp::Time last_target_{0, 0, RCL_ROS_TIME};
  bool have_target_{false};
  uint8_t sport_mode_{0};
  // Previous commanded velocities (for slew limiting). Accessed only from Tick.
  float prev_vx_{0.f};
  float prev_vyaw_{0.f};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2FollowController>();
  rclcpp::spin(node);
  // spin() returned — context is shutting down; on_shutdown already fired
  // the final StopMove. Fall through to clean exit.
  rclcpp::shutdown();
  return 0;
}
