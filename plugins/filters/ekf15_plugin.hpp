#pragma once

#include <rpp_cpp/plugin.hpp>

#include <rpp_localization/filters/extended_kalman_filter.hpp>

#include <memory>

#include "filter15_plugin_base.hpp"

// RPP entry point for the generic 15-state extended Kalman filter.
class Ekf15 final : public Filter15PluginBase
{
public:
  RPP_COMPONENTS(
    {"model", "rpp_localization::NavModel15"}
  )

  Ekf15() = default;
  ~Ekf15() override = default;

protected:
  std::unique_ptr<rpp_localization::FilterBase> create_filter(
    rpp_localization::ModelBase& model) override
  {
    return std::make_unique<rpp_localization::Ekf>(model);
  }
};
