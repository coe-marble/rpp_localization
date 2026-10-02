#pragma once

#include <rpp_localization/core/filter_common.hpp>
#include <rpp_localization/core/model_base.hpp>

#include <ostream>
#include <vector>

#define MB_DEBUG(msg) \
  if (get_debug()) { \
    *_debug_stream << msg; \
  }

namespace rpp_localization
{

class NavigationModelBase : public ModelBase
{
public:
  void reset();
  ~NavigationModelBase() override;

  [[nodiscard]] bool get_initialized_status() const;
  [[nodiscard]] const Eigen::MatrixXd& get_process_noise_covariance() const;
  [[nodiscard]] const std::vector<bool>& get_control_update_vector() const;

  void compute_dynamic_process_noise_covariance(
    const Eigen::VectorXd& state,
    Eigen::MatrixXd& covariance);

  void set_control_params(
    const std::vector<bool>& update_vector,
    DurationNs control_timeout,
    const std::vector<double>& acceleration_limits,
    const std::vector<double>& acceleration_gains,
    const std::vector<double>& deceleration_limits,
    const std::vector<double>& deceleration_gains);

  void set_dynamic_process_noise_covariance(bool enabled);
  void set_debug(bool debug, std::ostream* output_stream = nullptr);
  void set_process_noise_covariance(const Eigen::MatrixXd& process_noise_covariance);

protected:
  explicit NavigationModelBase(int state_dim);

  double compute_control_acceleration(
    double state,
    double control,
    double acceleration_limit,
    double acceleration_gain,
    double deceleration_limit,
    double deceleration_gain);
  void prepare_control(TimestampNs reference_time);

  bool initialized_;
  DurationNs control_timeout_;
  bool use_dynamic_process_noise_covariance_;

  std::vector<double> acceleration_gains_;
  std::vector<double> acceleration_limits_;
  std::vector<double> deceleration_gains_;
  std::vector<double> deceleration_limits_;
  std::vector<bool> control_update_vector_;

  Eigen::VectorXd control_acceleration_;
  Eigen::MatrixXd covariance_epsilon_;
  Eigen::MatrixXd dynamic_process_noise_covariance_;
  Eigen::MatrixXd identity_;
  Eigen::MatrixXd process_noise_covariance_;
};

}  // namespace rpp_localization
