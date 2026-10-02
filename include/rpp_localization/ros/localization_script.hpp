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

  void initialize()
  {
    context_.initialize();
    filter_ = context_.get_component<LocalizationFilter15>("filter");
    if (!filter_)
    {
      throw std::runtime_error(
        "LocalizationScript requires a LocalizationFilter15 component named 'filter'");
    }
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
