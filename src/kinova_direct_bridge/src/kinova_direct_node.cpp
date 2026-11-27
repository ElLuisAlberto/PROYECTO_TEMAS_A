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
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
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
      std::string error;
      if (!connect_usb(error)) RCLCPP_WARN(get_logger(), "USB connection failed: %s", error.c_str());
    } else { connected_ = true; }

    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cart_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_direct/cartesian_state", 10);
    fingers_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kinova_direct/finger_state", 10);
    fingers_ready_pub_ = create_publisher<std_msgs::msg::Bool>("kinova_direct/finger_ready", 10);
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>("kinova_direct/output_enabled", 10);
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("kinova_direct/usb_connected", 10);
    recovery_pub_ = create_publisher<std_msgs::msg::Bool>("kinova_direct/recovery_active", 10);
    permit_sub_ = create_subscription<std_msgs::msg::Bool>("kinova_direct/recovery_permit", 10, [this](const std_msgs::msg::Bool & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      gateway_permit_ = msg.data; gateway_at_ = std::chrono::steady_clock::now();
      if (!msg.data && (recovery_ || web_guard_)) halt_recovery();
    });
    web_permit_sub_ = create_subscription<std_msgs::msg::Bool>("kinova_direct/web_permit", 10, [this](const std_msgs::msg::Bool & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      web_permit_ = msg.data; web_permit_at_ = std::chrono::steady_clock::now();
    });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>("kinova_direct/recovery_heartbeat", 10, [this](const std_msgs::msg::Bool & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (msg.data && recovery_) recovery_at_ = std::chrono::steady_clock::now();
    });
    recovery_srv_ = create_service<std_srvs::srv::SetBool>("kinova_direct/recovery_mode", [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (closing_) { res->message = "La estación está cerrándose"; return; }
      if (returning_to_web_) { res->success = true; res->message = "Restableciendo control web…"; return; }
      if (req->data == recovery_) { res->success = true; res->message = "Sin cambios"; return; }
      const auto now = std::chrono::steady_clock::now();
      if (req->data && (!connected_ || !enabled_ || !gateway_permit_ || now - gateway_at_ > 300ms || now - joints_at_ > 250ms)) {
        res->message = "Se requiere salida armada, telemetría y sala operativa"; return;
      }
      if (!zero_motion()) { halt_recovery(); res->message = "No se pudo confirmar la parada"; return; }
      if (!req->data) {
        // Keep the exclusive recovery gate closed while zero commands flush
        // through the Cartesian adapter. The gateway must then explicitly
        // advertise READY before any new web command is accepted.
        returning_to_web_ = true; return_started_ = now; web_permit_ = false;
        res->success = true; res->message = "Restableciendo control web…"; return;
      }
      recovery_ = true; recovery_at_ = now; web_guard_ = false;
      res->success = true; res->message = "Recuperación activa; web pausada";
    });
    recovery_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("kinova_direct/recovery_velocity", 10, [this](const std_msgs::msg::Float64MultiArray & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (!recovery_ || returning_to_web_ || !enabled_ || !gateway_permit_ || now - gateway_at_ > 300ms || now - recovery_at_ > 300ms || now - joints_at_ > 250ms || msg.data.size() != 6) return;
      unsigned moving = 0;
      for (double v : msg.data) { if (!std::isfinite(v) || std::abs(v) > 5*pi/180 + 1e-9) return; if (std::abs(v) > 1e-9) ++moving; }
      if (moving > 1) return;
      for (int i : {1, 2}) {
        const double lo = (i == 1 ? 47.0 : 19.0)*pi/180, hi = (i == 1 ? 313.0 : 341.0)*pi/180;
        if ((msg.data[i] < 0 && current_joints_[i] <= lo + 3*pi/180) || (msg.data[i] > 0 && current_joints_[i] >= hi - 3*pi/180)) { if (!zero_motion()) halt_recovery(); return; }
      }
      if (!moving) { if (!zero_motion()) halt_recovery(); return; }
      std::copy(msg.data.begin(), msg.data.end(), requested_velocity_.begin());
      last_command_ = now; command_active_ = true;
    });
    shutdown_srv_ = create_service<std_srvs::srv::Trigger>("kinova_direct/prepare_shutdown", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
      std::lock_guard<std::mutex> guard(mutex_);
      // Once shutdown begins, no new control or reconnection may be accepted,
      // even if a stop or SDK close fails and the operator retries.
      closing_ = true; recovery_ = false; returning_to_web_ = false; web_guard_ = false;
      command_active_ = false; requested_velocity_.fill(0.0); mock_velocity_.fill(0.0);
      if (enabled_ || shutdown_stop_unconfirmed_) {
        shutdown_stop_unconfirmed_ = true;
        const int zero_result = zero_motion() ? NO_ERROR_KINOVA : -1;
        const int stop_result = mock_ ? NO_ERROR_KINOVA : StopControlAPI();
        if (zero_result != NO_ERROR_KINOVA || stop_result != NO_ERROR_KINOVA) {
          res->message = "No se pudo confirmar la parada; revise el brazo y vuelva a intentar";
          return;
        }
        shutdown_stop_unconfirmed_ = false; enabled_ = false; mode_ = 0;
      }
      if (api_open_) {
        const int result = CloseAPI();
        if (result != NO_ERROR_KINOVA) {
          res->message = "No se pudo confirmar el cierre del SDK: " + std::to_string(result);
          return;
        }
        api_open_ = false;
      }
      connected_ = false;
      res->success = true; res->message = "Salida detenida y conexión USB cerrada";
    });
    arm_intent_sub_ = create_subscription<geometry_msgs::msg::Twist>("/teleop/cartesian_twist", 10, [this](const geometry_msgs::msg::Twist & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      const std::array<double, 6> values{msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z};
      if (!arm_neutral_required_) return;
      const bool neutral = !finger_moving_ && std::abs(finger_requested_) < 1e-9 &&
          std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value) && std::abs(value) < 1e-9; });
      const auto now = std::chrono::steady_clock::now();
      if (!neutral) arm_zero_since_ = {};
      else if (arm_zero_since_ == std::chrono::steady_clock::time_point{}) arm_zero_since_ = now;
      else if (now - arm_zero_since_ >= 300ms) arm_neutral_required_ = false;
    });
    finger_sub_ = create_subscription<std_msgs::msg::Float64>("kinova_direct/finger_velocity", 10, [this](const std_msgs::msg::Float64 & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      // The web gripper is virtual. Physical USB must never accept this topic.
      if (!mock_) return;
      if (!std::isfinite(msg.data) || std::abs(msg.data) > 1.0) return;
      const auto now = std::chrono::steady_clock::now();
      if (std::abs(msg.data) < 1e-9) { finger_requested_ = 0.0; finger_neutral_required_ = false; finger_command_at_ = now; return; }
      if (closing_ || !enabled_ || !connected_ || !finger_hardware_ready_ || recovery_ || finger_neutral_required_ || now - fingers_at_ > 250ms ||
          (web_guard_ && (!web_permit_ || now - web_permit_at_ > 300ms))) return;
      finger_requested_ = msg.data;
      finger_command_at_ = now;
    });
    reconnect_srv_ = create_service<std_srvs::srv::Trigger>("kinova_direct/reconnect_usb", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
      std::lock_guard<std::mutex> guard(mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (closing_) { res->message = "La estación está cerrándose"; return; }
      if (connected_) { res->message = "El brazo ya está conectado"; res->success = true; return; }
      if (now - last_reconnect_ < 1s) { res->message = "Espere un segundo entre intentos"; return; }
      last_reconnect_ = now;
      std::string error;
      res->success = connect_usb(error);
      res->message = res->success ? "Brazo conectado; salida desarmada" : "No se pudo conectar: " + error;
    });
    disconnect_srv_ = create_service<std_srvs::srv::Trigger>("kinova_direct/disconnect_usb", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (mock_ || !connected_ || !api_open_) { res->message = "No hay brazo USB conectado"; return; }
      recovery_ = false; returning_to_web_ = false; web_guard_ = false; command_active_ = false;
      mock_velocity_.fill(0.0);
      if (enabled_) {
        const int zero_result = zero_motion() ? NO_ERROR_KINOVA : -1;
        const int stop_result = StopControlAPI();
        if (zero_result != NO_ERROR_KINOVA || stop_result != NO_ERROR_KINOVA) {
          res->message = "No se pudo confirmar la parada del brazo; revise el equipo";
          return;
        }
        enabled_ = false; mode_ = 0;
      }
      const int result = CloseAPI();
      api_open_ = false; connected_ = false;
      if (result != NO_ERROR_KINOVA) { res->message = "No se pudo confirmar el cierre USB: " + std::to_string(result); return; }
      res->success = true;
      res->message = "Brazo desconectado; puede retirar el cable USB";
    });
    enable_srv_ = create_service<std_srvs::srv::SetBool>("kinova_direct/enable_output", [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (closing_ && req->data) { res->success = false; res->message = "La estación está cerrándose"; return; }
      if (req->data && !connected_) { res->success = false; res->message = "Brazo USB desconectado"; return; }
      if (req->data == enabled_) { res->success = true; res->message = "unchanged"; return; }
      const bool finger_zero_ok = req->data || !finger_moving_ || zero_motion();
      int result = mock_ ? NO_ERROR_KINOVA : (req->data ? StartControlAPI() : StopControlAPI());
      if (!req->data && result == NO_ERROR_KINOVA) enabled_ = false;
      if (!finger_zero_ok || result != NO_ERROR_KINOVA) {
        if (!req->data) shutdown_stop_unconfirmed_ = true;
        res->success = false; res->message = "No se pudo confirmar la parada o el cambio de salida"; return;
      }
      if (!req->data) { recovery_ = false; returning_to_web_ = false; web_guard_ = false; finger_moving_ = false; finger_requested_ = 0.0; mock_finger_speed_ = 0.0; }
      enabled_ = req->data; command_active_ = false; mock_velocity_.fill(0.0); mode_ = 0; res->success = true; res->message = enabled_ ? "enabled" : "disabled";
    });
    joint_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("kinova_direct/joint_velocity", 10, [this](const std_msgs::msg::Float64MultiArray & msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (closing_ || !enabled_ || recovery_ || finger_moving_ || std::abs(finger_requested_) > 1e-9 || (web_guard_ && (!web_permit_ || std::chrono::steady_clock::now() - web_permit_at_ > 300ms))) return;
      if (msg.data.size() != 6) { RCLCPP_ERROR(get_logger(), "Expected six joint velocities in rad/s"); return; }
      if (arm_neutral_required_) return;
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
  bool stop_fingers() {
    if (!finger_moving_) return true;
    TrajectoryPoint zero; zero.InitStruct(); zero.Position.Type = CARTESIAN_VELOCITY;
    zero.Position.HandMode = VELOCITY_MODE;
    const bool ok = mock_ || SendAdvanceTrajectory(zero) == NO_ERROR_KINOVA;
    finger_moving_ = false; finger_requested_ = 0.0; mock_finger_speed_ = 0.0;
    return ok;
  }
  bool zero_motion() {
    command_active_ = false; requested_velocity_.fill(0.0); mock_velocity_.fill(0.0);
    if (!enabled_ || mock_) { finger_moving_ = false; finger_requested_ = 0.0; mock_finger_speed_ = 0.0; return true; }
    if (finger_moving_) return stop_fingers();
    TrajectoryPoint zero; zero.InitStruct(); zero.Position.Type = ANGULAR_VELOCITY; zero.Position.HandMode = HAND_NOMOVEMENT;
    return SendBasicTrajectory(zero) == NO_ERROR_KINOVA;
  }
  void halt_recovery() {
    const bool zero_ok = zero_motion();
    const int stop_result = (enabled_ && !mock_) ? StopControlAPI() : NO_ERROR_KINOVA;
    if (!zero_ok || stop_result != NO_ERROR_KINOVA) shutdown_stop_unconfirmed_ = true;
    enabled_ = false; recovery_ = false; returning_to_web_ = false; web_guard_ = false; mode_ = 0;
    RCLCPP_WARN(get_logger(), "Recovery interrupted; output disabled");
  }
  bool connect_usb(std::string & error) {
    enabled_ = false; command_active_ = false; mode_ = 0; mock_velocity_.fill(0.0);
    if (api_open_) { CloseAPI(); api_open_ = false; }
    try {
      check(InitAPI(), "InitAPI"); api_open_ = true;
      KinovaDevice devices[MAX_KINOVA_DEVICE]{}; int result = 0;
      int count = GetDevices(devices, result); check(result, "GetDevices");
      if (count != 1) throw std::runtime_error("Expected exactly one Kinova USB device, found " + std::to_string(count));
      check(SetActiveDevice(devices[0]), "SetActiveDevice");
      AngularPosition a{}; CartesianPosition c{};
      check(GetAngularPosition(a), "GetAngularPosition"); check(GetCartesianPosition(c), "GetCartesianPosition");
      connected_ = true; return true;
    } catch (const std::exception & e) {
      error = e.what(); connected_ = false;
      if (api_open_) { CloseAPI(); api_open_ = false; }
      return false;
    }
  }
  void disconnect_usb() {
    if (!connected_) return;
    if (enabled_) shutdown_stop_unconfirmed_ = true;
    finger_hardware_ready_ = false;
    connected_ = false; recovery_ = false; returning_to_web_ = false; web_guard_ = false; enabled_ = false; command_active_ = false; finger_moving_ = false; finger_requested_ = 0.0; mock_finger_speed_ = 0.0; mode_ = 0; mock_velocity_.fill(0.0);
    RCLCPP_ERROR(get_logger(), "USB connection lost; output disabled");
  }
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
        if ((recovery_ && (!gateway_permit_ || now - gateway_at_ > 300ms || now - recovery_at_ > 300ms || now - joints_at_ > 250ms)) || (web_guard_ && (!gateway_permit_ || now - gateway_at_ > 300ms))) halt_recovery();
        if (returning_to_web_ && now - return_started_ >= 1s) {
          returning_to_web_ = false; recovery_ = false; web_guard_ = true;
        }
        const bool finger_stale = !finger_hardware_ready_ || now - finger_command_at_ > 300ms || now - fingers_at_ > 250ms;
        bool finger_allowed = enabled_ && !closing_ && !recovery_ && !finger_neutral_required_ && !finger_stale && std::abs(finger_requested_) > 1e-9;
        if (finger_allowed) {
          const auto bounds = std::minmax_element(current_fingers_.begin(), current_fingers_.end());
          // Coordinated movement only. Leave a margin at the calibrated SDK
          // range ends; never continue closing when fingers diverge markedly.
          if ((finger_requested_ > 0 && (*bounds.second >= 6200 || *bounds.second - *bounds.first > 1200)) ||
              (finger_requested_ < 0 && *bounds.first <= 100)) {
            finger_allowed = false; finger_neutral_required_ = true;
          }
        }
        if (finger_allowed && finger_moving_ && now - finger_progress_at_ >= 2s) {
          for (size_t i = 0; i < 3; ++i)
            if (std::abs(current_fingers_[i] - finger_progress_[i]) < 2.0) { finger_allowed = false; finger_neutral_required_ = true; }
          finger_progress_ = current_fingers_; finger_progress_at_ = now;
        }
        if (finger_allowed) {
          if (!finger_moving_) {
            const double command = finger_requested_;
            if (!zero_motion() || !set_mode(2)) { halt_recovery(); finger_allowed = false; }
            else { finger_requested_ = command; arm_neutral_required_ = true; arm_zero_since_ = {}; finger_moving_ = true; finger_progress_ = current_fingers_; finger_progress_at_ = now; }
          }
          if (finger_allowed) {
            TrajectoryPoint point; point.InitStruct(); point.Position.Type = CARTESIAN_VELOCITY;
            point.Position.HandMode = VELOCITY_MODE;
            const double velocity = finger_requested_ * 150.0;
            point.Position.Fingers.Finger1 = velocity; point.Position.Fingers.Finger2 = velocity; point.Position.Fingers.Finger3 = velocity;
            if (!send(point)) { shutdown_stop_unconfirmed_ = true; halt_recovery(); }
            else mock_finger_speed_ = velocity;
          }
        } else if (finger_moving_) {
          const bool ok = stop_fingers();
          arm_neutral_required_ = true; arm_zero_since_ = {};
          if (!ok || finger_stale) { shutdown_stop_unconfirmed_ = shutdown_stop_unconfirmed_ || !ok; halt_recovery(); }
        }
        if (enabled_ && command_active_ && !finger_moving_) {
          if (now - last_command_ > 300ms) {
            TrajectoryPoint zero; zero.InitStruct(); zero.Position.Type = ANGULAR_VELOCITY;
            zero.Position.HandMode = HAND_NOMOVEMENT;
            if (!mock_) {
              const int zero_result = SendBasicTrajectory(zero);
              const int stop_result = StopControlAPI();
              if (zero_result != NO_ERROR_KINOVA || stop_result != NO_ERROR_KINOVA) {
                shutdown_stop_unconfirmed_ = true;
                RCLCPP_ERROR(get_logger(), "Timeout stop failed: zero=%d stop=%d", zero_result, stop_result);
              }
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
              if (!mock_ && StopControlAPI() != NO_ERROR_KINOVA) shutdown_stop_unconfirmed_ = true;
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
      if (finger_moving_) for (auto & value : mock_fingers_) value = std::clamp(value + mock_finger_speed_*dt, 0.0, 6800.0);
      if (mode_ == 1) for (size_t i=0; i<6; ++i) mock_joints_[i] += mock_velocity_[i]*dt;
      if (mode_ == 2) for (size_t i=0; i<6; ++i) mock_cart_[i] += mock_cart_velocity_[i]*dt;
    }
    sensor_msgs::msg::JointState state; state.header.stamp = get_clock()->now();
    for (int i=1; i<=6; ++i) state.name.push_back("j2n6s300_joint_" + std::to_string(i));
    std_msgs::msg::Float64MultiArray cart, fingers;
    if (mock_) { finger_hardware_ready_ = true; state.position.assign(mock_joints_.begin(), mock_joints_.end()); cart.data.assign(mock_cart_.begin(), mock_cart_.end()); for (double value : mock_fingers_) fingers.data.push_back(value * (80.0 / 6800.0) * pi / 180.0); }
    else if (connected_) {
      if (now - finger_status_at_ >= 2s) {
        Gripper status{}; const int result = GetGripperStatus(status);
        finger_hardware_ready_ = result == NO_ERROR_KINOVA;
        for (const auto & finger : status.Fingers)
          finger_hardware_ready_ = finger_hardware_ready_ && finger.IsFingerConnected != 0 && finger.IsFingerInit != 0;
        finger_status_at_ = now;
      }
      AngularPosition a{}; CartesianPosition c{};
      int ar = GetAngularPosition(a), cr = GetCartesianPosition(c);
      if (ar != NO_ERROR_KINOVA || cr != NO_ERROR_KINOVA) disconnect_usb();
      if (connected_ && ar == NO_ERROR_KINOVA) {
        auto & p = a.Actuators;
        const std::array<double, 6> degrees{p.Actuator1, p.Actuator2, p.Actuator3, p.Actuator4, p.Actuator5, p.Actuator6};
        for (size_t i = 0; i < degrees.size(); ++i) {
          double radians = degrees[i] * pi / 180;
          // Continuous joints may report more than one revolution. Preserve
          // their equivalent pose within the application's +/- 2*pi range;
          // never wrap the physically limited shoulder and elbow joints.
          if ((i == 0 || i >= 3) && std::isfinite(radians) && std::abs(radians) > 2*pi)
            radians = std::fmod(radians, 2*pi);
          state.position.push_back(radians);
        }
      }
      else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "GetAngularPosition failed: %d", ar);
      if (connected_ && cr == NO_ERROR_KINOVA) {
        auto & p=c.Coordinates; cart.data={p.X,p.Y,p.Z,p.ThetaX,p.ThetaY,p.ThetaZ};
        // SDK reports encoder counts. The official Kinova driver converts
        // 0..6800 counts to 0..80 degrees for JACO2 proximal joints.
        auto & f=c.Fingers;
        for (double value : {f.Finger1, f.Finger2, f.Finger3}) {
          if (!finger_hardware_ready_) break;
          if (!std::isfinite(value) || value < 0.0 || value > 6800.0) { fingers.data.clear(); break; }
          fingers.data.push_back(value * (80.0 / 6800.0) * pi / 180.0);
        }
      }
      else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "GetCartesianPosition failed: %d", cr);
    }
    if (state.position.size()==6 && std::all_of(state.position.begin(), state.position.end(), [](double x) { return std::isfinite(x); })) {
      std::copy(state.position.begin(), state.position.end(), current_joints_.begin());
      joints_at_ = now; joints_pub_->publish(state);
    }
    if (cart.data.size()==6) cart_pub_->publish(cart);
    if (fingers.data.size()==3) {
      for (size_t i = 0; i < 3; ++i) current_fingers_[i] = fingers.data[i] * 180.0 / pi * (6800.0 / 80.0);
      fingers_at_ = now; fingers_pub_->publish(fingers);
    }
    std_msgs::msg::Bool enabled; enabled.data=enabled_; enabled_pub_->publish(enabled);
    std_msgs::msg::Bool connected; connected.data=connected_; connected_pub_->publish(connected);
    std_msgs::msg::Bool recovery; recovery.data=recovery_; recovery_pub_->publish(recovery);
    std_msgs::msg::Bool finger_ready; finger_ready.data=finger_hardware_ready_; fingers_ready_pub_->publish(finger_ready);
  }
  std::mutex mutex_;
  std::atomic<bool> stop_thread_{false};
  std::thread control_thread_;
  std::array<double, 6> requested_velocity_{};
  bool closing_{false}, shutdown_stop_unconfirmed_{false};
  bool finger_hardware_ready_{false}, finger_moving_{false}, finger_neutral_required_{false}, arm_neutral_required_{false};
  std::chrono::steady_clock::time_point finger_status_at_{};
  double finger_requested_{0.0}, mock_finger_speed_{0.0};
  std::array<double, 3> current_fingers_{}, finger_progress_{}, mock_fingers_{2000.0, 2000.0, 2000.0};
  std::chrono::steady_clock::time_point fingers_at_{}, finger_command_at_{}, finger_progress_at_{}, arm_zero_since_{};
  bool recovery_{false}, gateway_permit_{false}, returning_to_web_{false}, web_guard_{false}, web_permit_{false};
  std::chrono::steady_clock::time_point return_started_{}, web_permit_at_{};
  std::array<double, 6> current_joints_{};
  std::chrono::steady_clock::time_point gateway_at_{}, recovery_at_{}, joints_at_{};
  bool connected_{false};
  std::chrono::steady_clock::time_point last_reconnect_{};
  bool mock_{true}, api_open_{false}, enabled_{false}, command_active_{false}; int lock_{-1}, mode_{0};
  std::array<double,6> mock_joints_{0.2, 2.4, 2.7, 0.4, 0.8, 0.1}, mock_velocity_{}, mock_cart_{}, mock_cart_velocity_{};
  std::chrono::steady_clock::time_point last_, last_command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cart_pub_, fingers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_, recovery_pub_, fingers_ready_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_, heartbeat_sub_, web_permit_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr recovery_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recovery_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_srv_, disconnect_srv_, shutdown_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr finger_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr arm_intent_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DirectBridge>()); }
  catch (const std::exception & e) { RCLCPP_FATAL(rclcpp::get_logger("kinova_direct_bridge"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
