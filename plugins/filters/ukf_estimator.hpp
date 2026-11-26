#pragma once
#include "shared_estimator_base.hpp"
#include <rpp_localization/estimation/ukf.hpp>

template<bool CollectMetrics>
class UkfBase : public SharedEstimatorBase<CollectMetrics>
{
public:
  using ParameterDescription = typename SharedEstimatorBase<CollectMetrics>::ParameterDescription;
  using FieldsDesc = typename SharedEstimatorBase<CollectMetrics>::FieldsDesc;
  RPP_COMPONENTS({"model", "rpp_interfaces::Model"})
  RPP_PARAMETERS(
    ParameterDescription::template create<std::vector<FieldsDesc>>("measurements", std::vector<FieldsDesc>{}),
    ParameterDescription::template create<std::vector<double>>("mahalanobis_thresholds", std::vector<double>{}),
    ParameterDescription::template create<std::int64_t>("history_capacity", 512),
    ParameterDescription::template create<std::string>("replay_noise_policy", "reject_split"),
    ParameterDescription::template create<double>("alpha", 0.001),
    ParameterDescription::template create<double>("kappa", 0.0),
    ParameterDescription::template create<double>("beta", 2.0),
    ParameterDescription::template create<bool>("legacy_ukf_correction", true)
  )
protected:
  std::uint8_t required_process_features() const override
  {return rpp_interfaces::contract::MODEL_PREDICT;}
  std::uint8_t required_observation_features() const override
  {return rpp_interfaces::contract::MODEL_OBSERVE;}
  std::unique_ptr<rpp_localization::estimation::GaussianFilter> create_filter(
    const rpp_localization::estimation::ProcessModel& model,
    const std::vector<rpp_localization::estimation::ObservationBinding>& observations,
    const rpp::ComponentContext& context) override
  {
    namespace e = rpp_localization::estimation;
    e::UkfOptions options;
    options.alpha = context.get_parameter<double>("alpha");
    options.kappa = context.get_parameter<double>("kappa");
    options.beta = context.get_parameter<double>("beta");
    options.correction = context.get_parameter<bool>("legacy_ukf_correction") ?
      e::UkfCorrection::legacy : e::UkfCorrection::innovation;
    return std::make_unique<e::UkfImplementation<CollectMetrics>>(model, observations, options, this->metrics_sink());
  }
};

class Ukf final : public UkfBase<false> {};
