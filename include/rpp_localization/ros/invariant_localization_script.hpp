#ifndef RPP_LOCALIZATION__ROS__INVARIANT_LOCALIZATION_SCRIPT_HPP_
#define RPP_LOCALIZATION__ROS__INVARIANT_LOCALIZATION_SCRIPT_HPP_

#include <stdexcept>

#include <rpp_cpp/context.hpp>
#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_localization/InvariantFilter.hpp>

namespace rpp_localization
{

class InvariantLocalizationScript final
{
public:
  RPP_COMPONENTS(
    {"filter", "rpp_localization::InvariantFilter"}
  )

  explicit InvariantLocalizationScript(const rpp::ComponentContext & context)
  : context_(context)
  {
  }

  void initialize(bool activate_on_start = true)
  {
    if (!context_.get_subcomponent_context("filter").get_instance<InvariantFilter>())
    {
      throw std::runtime_error(
        "InvariantLocalizationScript requires an InvariantFilter component named 'filter'");
    }
    if (activate_on_start)
    {
      context_.activate_subcomponents().require_success(
        "InvariantLocalizationScript component activation");
    }
  }

  [[nodiscard]] bool is_active() const
  {
    return static_cast<bool>(context_.get_active_component<InvariantFilter>("filter"));
  }

private:
  const rpp::ComponentContext & context_;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__ROS__INVARIANT_LOCALIZATION_SCRIPT_HPP_
