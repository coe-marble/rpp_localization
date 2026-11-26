#include <memory>

#include <rpp_localization/estimation/ukf.hpp>
#include <rpp_localization/models/native_navigation15.hpp>

namespace rpp_localization::test
{
std::unique_ptr<estimation::ProcessModel> make_peer_model()
{
  return std::make_unique<NativeNavigationModel15>();
}

std::unique_ptr<estimation::GaussianFilter> make_peer_filter(
  const estimation::ProcessModel& model)
{
  return std::make_unique<estimation::Ukf>(
    model, std::vector<estimation::ObservationBinding>{});
}
}  // namespace rpp_localization::test
