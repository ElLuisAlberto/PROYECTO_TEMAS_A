#include <array>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <Kinova.API.USBCommandLayerUbuntu.h>

using namespace std::chrono_literals;
constexpr double pi = 3.14159265358979323846;
class DirectBridge : public rclcpp::Node {
 public:
  DirectBridge() : Node("kinova_direct_bridge") {
    mock_ = declare_parameter<bool>("mock_hardware", true);
    if (!mock_) {
      lock_ = open("/tmp/thesis_kinova_jaco2_usb.lock", O_CREAT | O_RDWR | O_CLOEXEC, 0660);
      if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) != 0) throw std::runtime_error("Kinova USB is already in use or lock unavailable");
      check(InitAPI(), "InitAPI"); api_open_ = true;
      try {
        KinovaDevice devices[MAX_KINOVA_DEVICE]{}; int result = 0;
        int count = GetDevices(devices, result); check(result, "GetDevices");
        if (count != 1) throw std::runtime_error("Expected exactly one Kinova USB device, found " + std::to_string(count));
        check(SetActiveDevice(devices[0]), "SetActiveDevice");
        QuickStatus status{};
        int status_result = GetQuickStatus(status);
        if (status_result == NO_ERROR_KINOVA) {
          RCLCPP_INFO(get_logger(), "SDK QuickStatus: control_enable=%u active_module=%u retract=%u cartesian_fault=%u", static_cast<unsigned>(status.ControlEnableStatus), static_cast<unsigned>(status.ControlActiveModule), static_cast<unsigned>(status.RetractType), static_cast<unsigned>(status.CartesianFaultState));
        } else {
          RCLCPP_WARN(get_logger(), "GetQuickStatus failed: %d", status_result);
        }
      } catch (...) { CloseAPI(); api_open_ = false; close(lock_); lock_ = -1; throw; }
    }
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cart_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_direct/cartesian_state", 10);
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>("kinova_direct/output_enabled", 10);
    enable_srv_ = create_service<std_srvs::srv::SetBool>("kinova_direct/enable_output", [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (req->data == enabled_) { res->success = true; res->message = "unchanged"; return; }
      int result = mock_ ? NO_ERROR_KINOVA : (req->data ? StartControlAPI() : StopControlAPI());
      if (result != NO_ERROR_KINOVA) { res->success = false; res->message = "SDK error " + std::to_string(result); return; }
      enabled_ = req->data; command_active_ = false; mock_velocity_.fill(0.0); mode_ = 0; res->success = true; res->message = enabled_ ? "enabled" : "disabled";
    });
    joint_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("kinova_direct/joint_velocity", 10, [this](const std_msgs::msg::Float64MultiArray & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!enabled_) { RCLCPP_WARN(get_logger(), "Output disabled"); return; }
      if (msg.data.size() != 6) { RCLCPP_ERROR(get_logger(), "Expected six joint velocities in rad/s"); return; }
      for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(msg.data[i])) {
          RCLCPP_ERROR(get_logger(), "Non-finite joint velocity"); return;
        }
      }
      std::copy(msg.data.begin(), msg.data.end(), requested_velocity_.begin());
      last_command_ = std::chrono::steady_clock::now();
      command_active_ = true;
    });
    last_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(50ms, [this]() { publish_state(); });
    control_thread_ = std::thread(&DirectBridge::control_loop, this);
    RCLCPP_INFO(get_logger(), "Direct bridge started: mock=%s, output disabled", mock_ ? "true" : "false");
  }
  ~DirectBridge() override {
    stop_thread_.store(true);
    if (control_thread_.joinable()) control_thread_.join();
    if (api_open_) { if (enabled_) StopControlAPI(); CloseAPI(); }
    if (lock_ >= 0) close(lock_);
  }
 private:
  static void check(int result, const char *name) { if (result != NO_ERROR_KINOVA) throw std::runtime_error(std::string(name) + " failed: " + std::to_string(result)); }
  bool set_mode(int mode) {
    if (mode_ == mode) return true;
    int result = mock_ ? NO_ERROR_KINOVA : (mode == 1 ? SetAngularControl() : SetCartesianControl());
    if (result != NO_ERROR_KINOVA) { RCLCPP_ERROR(get_logger(), "Set control mode failed: %d", result); return false; }
    mode_ = mode; return true;
  }
  bool send(TrajectoryPoint & point) {
    int result = mock_ ? NO_ERROR_KINOVA : (point.Position.Type == CARTESIAN_VELOCITY ? SendAdvanceTrajectory(point) : SendBasicTrajectory(point));
    if (result != NO_ERROR_KINOVA) { RCLCPP_ERROR(get_logger(), "Sending trajectory failed: %d", result); return false; }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "SDK accepted %s velocity command", point.Position.Type == CARTESIAN_VELOCITY ? "Cartesian" : "joint");
    return true;
  }
  void control_loop() {
    auto next = std::chrono::steady_clock::now();
    auto report_at = next + 1s;
    unsigned sent = 0;
    while (!stop_thread_.load() && rclcpp::ok()) {
      next += 10ms;
      {
        // Serialize SDK calls and state transitions: a stale command cannot
        // be transmitted after the output has been disarmed.
        std::lock_guard<std::mutex> guard(mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (enabled_ && command_active_) {
          if (now - last_command_ > 300ms) {
            TrajectoryPoint zero; zero.InitStruct(); zero.Position.Type = ANGULAR_VELOCITY;
            zero.Position.HandMode = HAND_NOMOVEMENT;
            if (!mock_) {
              const int zero_result = SendBasicTrajectory(zero);
              const int stop_result = StopControlAPI();
              if (zero_result != NO_ERROR_KINOVA || stop_result != NO_ERROR_KINOVA)
                RCLCPP_ERROR(get_logger(), "Timeout stop failed: zero=%d stop=%d", zero_result, stop_result);
            }
            mock_velocity_.fill(0.0); enabled_ = false; command_active_ = false; mode_ = 0;
            RCLCPP_WARN(get_logger(), "Command timeout: zero requested and output disabled");
          } else {
            TrajectoryPoint point; point.InitStruct(); point.Position.Type = ANGULAR_VELOCITY;
            point.Position.HandMode = HAND_NOMOVEMENT;
            auto & a = point.Position.Actuators;
            a.Actuator1=requested_velocity_[0]*180/pi; a.Actuator2=requested_velocity_[1]*180/pi;
            a.Actuator3=requested_velocity_[2]*180/pi; a.Actuator4=requested_velocity_[3]*180/pi;
            a.Actuator5=requested_velocity_[4]*180/pi; a.Actuator6=requested_velocity_[5]*180/pi;
            if (set_mode(1) && send(point)) {
              mock_velocity_ = requested_velocity_; ++sent;
            } else {
              if (!mock_) StopControlAPI();
              mock_velocity_.fill(0.0); enabled_ = false; command_active_ = false; mode_ = 0;
              RCLCPP_ERROR(get_logger(), "Control output failed; output disabled");
            }
          }
        }
        if (now >= report_at) {
          if (sent) RCLCPP_INFO(get_logger(), "Joint velocity transmissions in last interval: %u", sent);
          sent = 0; report_at = now + 1s;
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (now > next + 50ms) next = now;
      std::this_thread::sleep_until(next);
    }
  }
  void publish_state() {
    std::lock_guard<std::mutex> guard(mutex_);
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now-last_).count(); last_=now;
    if (mock_ && enabled_) {
      if (mode_ == 1) for (size_t i=0; i<6; ++i) mock_joints_[i] += mock_velocity_[i]*dt;
      if (mode_ == 2) for (size_t i=0; i<6; ++i) mock_cart_[i] += mock_cart_velocity_[i]*dt;
    }
    sensor_msgs::msg::JointState state; state.header.stamp = get_clock()->now();
    for (int i=1; i<=6; ++i) state.name.push_back("j2n6s300_joint_" + std::to_string(i));
    std_msgs::msg::Float64MultiArray cart;
    if (mock_) { state.position.assign(mock_joints_.begin(), mock_joints_.end()); cart.data.assign(mock_cart_.begin(), mock_cart_.end()); }
    else {
      AngularPosition a{}; CartesianPosition c{};
      int ar = GetAngularPosition(a), cr = GetCartesianPosition(c);
      if (ar == NO_ERROR_KINOVA) { auto & p=a.Actuators; for (auto value : {p.Actuator1,p.Actuator2,p.Actuator3,p.Actuator4,p.Actuator5,p.Actuator6}) state.position.push_back(value*pi/180); }
      else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "GetAngularPosition failed: %d", ar);
      if (cr == NO_ERROR_KINOVA) { auto & p=c.Coordinates; cart.data={p.X,p.Y,p.Z,p.ThetaX,p.ThetaY,p.ThetaZ}; }
      else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "GetCartesianPosition failed: %d", cr);
    }
    if (state.position.size()==6) joints_pub_->publish(state);
    if (cart.data.size()==6) cart_pub_->publish(cart);
    std_msgs::msg::Bool enabled; enabled.data=enabled_; enabled_pub_->publish(enabled);
  }
  std::mutex mutex_;
  std::atomic<bool> stop_thread_{false};
  std::thread control_thread_;
  std::array<double, 6> requested_velocity_{};
  bool mock_{true}, api_open_{false}, enabled_{false}, command_active_{false}; int lock_{-1}, mode_{0};
  std::array<double,6> mock_joints_{0.2, 2.4, 2.7, 0.4, 0.8, 0.1}, mock_velocity_{}, mock_cart_{}, mock_cart_velocity_{};
  std::chrono::steady_clock::time_point last_, last_command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cart_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DirectBridge>()); }
  catch (const std::exception & e) { RCLCPP_FATAL(rclcpp::get_logger("kinova_direct_bridge"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
