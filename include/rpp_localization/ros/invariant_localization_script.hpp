#ifndef RPP_LOCALIZATION__ROS__INVARIANT_LOCALIZATION_SCRIPT_HPP_
#define RPP_LOCALIZATION__ROS__INVARIANT_LOCALIZATION_SCRIPT_HPP_

#include <memory>
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

  void initialize()
  {
    context_.initialize();
    filter_ = context_.get_component<InvariantFilter>("filter");
    if (!filter_)
    {
      throw std::runtime_error(
        "InvariantLocalizationScript requires an InvariantFilter component named 'filter'");
    }
  }

  [[nodiscard]] const std::shared_ptr<InvariantFilter> & filter() const noexcept
  {
    return filter_;
  }

private:
  const rpp::ComponentContext & context_;
  std::shared_ptr<InvariantFilter> filter_;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__ROS__INVARIANT_LOCALIZATION_SCRIPT_HPP_
