#pragma once

#include <rpp_cpp/plugin.hpp>

#include <rpp_localization/core/filter_common.hpp>
#include <rpp_localization/filters/unscented_kalman_filter.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "filter15_plugin_base.hpp"

// RPP entry point for the generic 15-state unscented Kalman filter.
class Ukf15 final : public Filter15PluginBase
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;

  RPP_COMPONENTS(
    {"model", "rpp_localization::NavModel15"}
  )

  RPP_PARAMETERS(
    ParameterDescription::create<std::vector<double>>(
      "process_noise_covariance", std::vector<double>{}),
    ParameterDescription::create<bool>("dynamic_process_noise_covariance", false),
    ParameterDescription::create<double>("alpha", 0.001),
    ParameterDescription::create<double>("kappa", 0.0),
    ParameterDescription::create<double>("beta", 2.0)
  )

  Ukf15() = default;
  ~Ukf15() override = default;

  void initialize(const rpp::ComponentContext& context) override
  {
    load_configuration(context);
    Filter15PluginBase::initialize(context);
  }

protected:
  std::unique_ptr<rpp_localization::FilterBase> create_filter(
    rpp_localization::ModelBase& model) override
  {
    auto filter = std::make_unique<rpp_localization::Ukf>(model);
    filter->initialize_runtime(
      process_noise_covariance_, dynamic_process_noise_covariance_, alpha_, kappa_, beta_);
    return filter;
  }

private:
  static constexpr std::size_t k_state_size = rpp_localization::STATE_SIZE;
  static constexpr std::size_t k_covariance_size = k_state_size * k_state_size;

  void load_configuration(const rpp::ComponentContext& context)
  {
    const auto covariance_values =
      context.get_parameter<std::vector<double>>("process_noise_covariance");
    dynamic_process_noise_covariance_ =
      context.get_parameter<bool>("dynamic_process_noise_covariance");
    alpha_ = context.get_parameter<double>("alpha");
    kappa_ = context.get_parameter<double>("kappa");
    beta_ = context.get_parameter<double>("beta");

    if (!std::isfinite(alpha_) || !std::isfinite(kappa_) || !std::isfinite(beta_))
    {
      throw std::invalid_argument("UKF alpha, kappa, and beta must be finite");
    }
    const double sigma_scale = alpha_ * alpha_ * (static_cast<double>(k_state_size) + kappa_);
    if (alpha_ <= 0.0 || !std::isfinite(sigma_scale) ||
      sigma_scale <= std::numeric_limits<double>::min())
    {
      throw std::invalid_argument("UKF alpha and kappa must produce a positive sigma scale");
    }

    process_noise_covariance_.setZero(k_state_size, k_state_size);
    if (covariance_values.empty())
    {
      return;
    }
    if (covariance_values.size() != k_covariance_size)
    {
      throw std::invalid_argument("UKF process_noise_covariance must contain 225 values");
    }

    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        const double value = covariance_values[row * k_state_size + column];
        if (!std::isfinite(value))
        {
          throw std::invalid_argument("UKF process_noise_covariance must be finite");
        }
        process_noise_covariance_(static_cast<Eigen::Index>(row),
          static_cast<Eigen::Index>(column)) = value;
      }
      if (process_noise_covariance_(static_cast<Eigen::Index>(row),
        static_cast<Eigen::Index>(row)) < 0.0)
      {
        throw std::invalid_argument(
                "UKF process_noise_covariance must have a non-negative diagonal");
      }
    }
  }

  rpp_localization::CovarianceMatrix process_noise_covariance_;
  bool dynamic_process_noise_covariance_{false};
  double alpha_{0.001};
  double kappa_{0.0};
  double beta_{2.0};
};
