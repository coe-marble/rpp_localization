#ifndef RPP_LOCALIZATION__ROS__LOCALIZATION_SCRIPT_HPP_
#define RPP_LOCALIZATION__ROS__LOCALIZATION_SCRIPT_HPP_

#include <map>
#include <memory>
#include <stdexcept>

#include <rpp_cpp/context.hpp>
#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_localization/LocalizationFilter15.hpp>

namespace rpp_localization
{

class LocalizationScript final
{
public:
  RPP_COMPONENTS(
    {"filter", "rpp_localization::LocalizationFilter15"}
  )

  explicit LocalizationScript(const rpp::ComponentContext & context)
  : context_(context)
  {
  }

  void initialize(bool activate_on_start = true)
  {
    filter_ = context_.get_subcomponent_context("filter").get_instance<LocalizationFilter15>();
    if (!filter_)
    {
      throw std::runtime_error(
        "LocalizationScript requires a LocalizationFilter15 component named 'filter'");
    }
    if (activate_on_start)
    {
      context_.activate_subcomponents().require_success(
        "LocalizationScript component activation");
    }
  }

  [[nodiscard]] bool is_active() const
  {
    return context_.get_subcomponent_context("filter").is_active();
  }

  [[nodiscard]] const std::shared_ptr<LocalizationFilter15> & filter() const noexcept
  {
    return filter_;
  }

private:
  const rpp::ComponentContext & context_;
  std::shared_ptr<LocalizationFilter15> filter_;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__ROS__LOCALIZATION_SCRIPT_HPP_
