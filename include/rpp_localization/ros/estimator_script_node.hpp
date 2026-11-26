#pragma once

#include <rpp_localization/estimation/localization_script.hpp>
#include <rpp_localization/estimation/prepared_queue.hpp>
#include <rpp_localization/estimation/replay_driver.hpp>
#include <rpp_localization/ros/estimator_node_helpers.hpp>
#include <rpp_localization/ros/benchmark_timing.hpp>
#include <rpp_localization/ros/worker_metrics.hpp>
#include <rpp_ros_interfaces/msg/estimator_layout.hpp>
#include <rpp_ros_interfaces/msg/estimator_measurement.hpp>
#include <rpp_ros_interfaces/msg/estimator_prediction.hpp>
#include <rpp_ros_interfaces/msg/estimator_estimate.hpp>
#include <rpp_ros_interfaces/msg/estimator_input.hpp>
#include <rpp_ros_interfaces/msg/estimator_status.hpp>
#include <rpp_ros_interfaces/srv/reset_estimate.hpp>
#include <rpp_ros_interfaces/msg/numeric_sample.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_ros2/slot_manager_ros.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <map>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <cstdint>
#include <string>
#include <vector>

namespace rpp_localization
{
class NavigationInterface;
struct EstimatorNodeDefaults
{
  std::string configuration;
  std::string script_name{"localization_generic"};
  bool expose_navigation{false};
};

// ROS ingress and lifecycle run on a SingleThreadedExecutor. A separate worker
// owns live numeric execution; the ROS executor services the idle replay replica.
// Prepared SPSC queues cross that boundary; lifecycle operations quiesce both
// owners before touching layouts or component state.
// No navigation dimensions, state roles or concrete filter types live here.
class EstimatorScriptNode final : public rclcpp::Node
{
public:
  using Estimate = rpp_ros_interfaces::msg::EstimatorEstimate;
  using PredictionPolicy = std::function<std::uint8_t(std::uint64_t, std::uint64_t,
    const Estimate&, rpp::List<double>&, rpp::List<double>&)>;

  explicit EstimatorScriptNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    EstimatorNodeDefaults defaults = {});
  ~EstimatorScriptNode() override;

  const rpp_interfaces::FieldLayout& state_layout() const {return script_->state_layout();}
  const rpp_interfaces::FieldLayout& input_layout() const {return script_->input_layout();}
  const std::vector<rpp_interfaces::FieldLayout>& measurement_layouts() const
  {return script_->measurement_layouts();}
  std::uint64_t layout_revision() const {return revision_;}
  void set_prediction_policy(PredictionPolicy policy);
  std::uint8_t submit_measurement(std::uint32_t index, const std::vector<double>& values,
    const std::vector<double>& covariance, std::uint64_t time);
  std::uint8_t submit_prediction(const std::vector<double>& input, const std::vector<double>& q,
    std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt);
  std::uint8_t submit_input(const std::vector<double>& values, std::uint64_t time, std::uint64_t input_dt);

private:
  friend class NavigationInterface;
  struct Port
  {
    rpp::List<double> values, covariance;
    std::vector<std::size_t> initial_route;
    std::vector<std::size_t> initial_covariance_route;
  };
  enum class EventKind {Measurement, Prediction, Input};
  struct Event
  {
    Event(std::size_t width, std::size_t square) : values(width), covariance(square) {}
    EventKind kind{EventKind::Measurement};
    std::uint64_t epoch{0}, time{0}, dt{0}, input_dt{0};
    std::uint64_t enqueued_ns{0};
    std::uint32_t index{0};
    std::size_t width{0}, square{0};
    std::vector<double> values, covariance;
  };
  struct Output
  {
    Output(std::size_t width, std::size_t square) : state(width), covariance(square) {}
    std::uint64_t epoch{0}, time{0}, replay_revision{0};
    std::uint8_t code{0};
    std::vector<double> state, covariance;
  };
  class ManagementPause
  {
  public:
    explicit ManagementPause(EstimatorScriptNode& node) : node_(node) {node_.enter_management();}
    ~ManagementPause() {node_.leave_management();}
  private:
    EstimatorScriptNode& node_;
  };
  struct StateRoute
  {
    rpp_ros_interfaces::msg::NumericSample message;
    std::vector<std::size_t> indices;
  };
  void build_runtime();
  void prepare();
  void tick();
  void run_worker();
  void run_replay();
  bool adopt_replay();
  void cancel_replay();
  void enter_management();
  void leave_management();
  std::uint8_t process(const Event& event);
  std::uint8_t enqueue(EventKind kind, std::uint32_t index, const std::vector<double>& values,
    const std::vector<double>& covariance, std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt);
  bool capture_estimate(std::uint8_t code, bool publish = true);
  void status(std::uint8_t code, const std::string& message = "");
  void publish_estimate();
  std::uint8_t begin(std::uint64_t time, std::optional<std::uint32_t> measurement = {});
  std::uint8_t predict_to(std::uint64_t time);
  void reset(const rpp_ros_interfaces::srv::ResetEstimate::Request& request,
    rpp_ros_interfaces::srv::ResetEstimate::Response& response, bool preserve_enable = false);
  void slot_changed(const rpp_ros2::SlotOperation& operation);
  void before_slot_operation(const rpp_ros2::SlotOperation& operation);
  void observe_estimator() noexcept;

  std::unique_ptr<rpp::RppDataManager> data_manager_;
  EstimatorNodeDefaults defaults_;
  std::unique_ptr<rpp::ConcurrentScriptRuntime> runtime_;
  std::unique_ptr<GenericLocalizationScript> script_;
  std::unique_ptr<rpp_ros2::SlotManagerRos<rpp::ConcurrentScriptRuntime>> slots_;
  std::unique_ptr<NavigationInterface> navigation_;
  PredictionPolicy prediction_policy_;
  rpp::List<double> initial_state_, initial_covariance_, reset_state_, reset_covariance_;
  rpp::List<double> input_, prediction_input_, density_, q_;
  std::vector<Port> ports_;
  std::vector<StateRoute> state_routes_;
  std::map<std::string, rpp_interfaces::FieldLayout> state_interface_definitions_;
  Estimate estimate_, worker_estimate_;
  estimation::PreparedQueue<Event> incoming_;
  estimation::PreparedQueue<Output> outgoing_;
  std::mutex execution_mutex_, wake_mutex_;
  std::condition_variable wake_;
  std::unique_lock<std::mutex> management_lock_;
  std::size_t management_depth_{0}, queue_capacity_{128};
  std::atomic<bool> paused_{false}, stop_{false}, replay_ready_{false}, replay_estimate_pending_{false};
  // Functional freshness: the ROS wall timer must publish an adopted posterior
  // even if simulated time stops before the next normal publication timer.
  std::atomic<std::uint64_t> replay_revision_{0};
  std::uint64_t published_replay_revision_{0};
  std::atomic<std::uint8_t> worker_status_{0};
  std::thread worker_;
  BenchmarkTiming benchmark_;
  WorkerMetrics worker_metrics_;
  estimation::ReplayDriver replay_driver_;
  bool replay_on_ros_{true};
  std::size_t replay_capacity_{4096}, replay_events_per_tick_{64};
  std::string replay_plugin_hint_;
  std::string estimator_instance_id_;
  std::chrono::nanoseconds worker_period_{33'333'333};
  std::uint64_t epoch_{0};
  std::uint64_t epoch_start_{0};
  std::uint64_t revision_{0}, time_{0}, input_dt_{0}, input_time_{0}, previous_now_{0};
  std::uint64_t input_timeout_{0}, sensor_timeout_{0};
  bool configured_{false}, initialized_{false}, enabled_{true}, input_ready_{false};
  bool primed_{false}, routes_dirty_{false}, activate_on_start_{true};
  bool predict_current_{true}, initialize_from_measurement_{true};
  rclcpp::Publisher<rpp_ros_interfaces::msg::EstimatorLayout>::SharedPtr layout_publisher_;
  rclcpp::Publisher<Estimate>::SharedPtr estimate_publisher_;
  rclcpp::Publisher<rpp_ros_interfaces::msg::NumericSample>::SharedPtr state_publisher_;
  rclcpp::Publisher<rpp_ros_interfaces::msg::EstimatorStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<rpp_ros_interfaces::msg::EstimatorMeasurement>::SharedPtr measurement_subscription_;
  rclcpp::Subscription<rpp_ros_interfaces::msg::EstimatorPrediction>::SharedPtr prediction_subscription_;
  rclcpp::Subscription<rpp_ros_interfaces::msg::EstimatorInput>::SharedPtr input_subscription_;
  rclcpp::Service<rpp_ros_interfaces::srv::ResetEstimate>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr benchmark_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr replay_timer_;
};
}  // namespace rpp_localization
