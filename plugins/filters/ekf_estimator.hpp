#pragma once
#include "shared_estimator_base.hpp"
#include <rpp_localization/estimation/ekf.hpp>

template<bool CollectMetrics>
class EkfBase : public SharedEstimatorBase<CollectMetrics>
{
public:
  using ParameterDescription = typename SharedEstimatorBase<CollectMetrics>::ParameterDescription;
  using FieldsDesc = typename SharedEstimatorBase<CollectMetrics>::FieldsDesc;
  RPP_COMPONENTS({"model", "rpp_interfaces::Model"})
  RPP_PARAMETERS(
    ParameterDescription::template create<std::vector<FieldsDesc>>("measurements", std::vector<FieldsDesc>{}),
    ParameterDescription::template create<std::vector<double>>("mahalanobis_thresholds", std::vector<double>{}),
    ParameterDescription::template create<std::int64_t>("history_capacity", 512),
    ParameterDescription::template create<std::string>("replay_noise_policy", "reject_split")
  )
protected:
  std::uint8_t required_process_features() const override
  {return rpp_interfaces::contract::MODEL_PREDICT | rpp_interfaces::contract::MODEL_LINEARIZE;}
  std::uint8_t required_observation_features() const override
  {return rpp_interfaces::contract::MODEL_OBSERVE | rpp_interfaces::contract::MODEL_OBSERVATION_JACOBIAN;}
  std::unique_ptr<rpp_localization::estimation::GaussianFilter> create_filter(
    const rpp_localization::estimation::ProcessModel& model,
    const std::vector<rpp_localization::estimation::ObservationBinding>& observations,
    const rpp::ComponentContext&) override
  {
    namespace e = rpp_localization::estimation;
    return std::make_unique<e::EkfImplementation<CollectMetrics>>(model, observations, this->metrics_sink());
  }
};

class Ekf final : public EkfBase<false> {};
