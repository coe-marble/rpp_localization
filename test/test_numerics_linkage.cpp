#include <memory>

#include <gtest/gtest.h>

#include <rpp_localization/estimation/ekf.hpp>
#include <rpp_localization/estimation/ukf.hpp>
#include <rpp_localization/models/native_navigation15.hpp>

namespace rpp_localization::test
{
std::unique_ptr<estimation::ProcessModel> make_peer_model();
std::unique_ptr<estimation::GaussianFilter> make_peer_filter(
  const estimation::ProcessModel& model);

TEST(NumericsLinkage, LinksGenericFiltersAndNavigationAcrossTranslationUnits)
{
  namespace e = estimation;
  auto peer_model = make_peer_model();
  auto peer_filter = make_peer_filter(*peer_model);
  NativeNavigationModel15 local_model;
  e::Ekf ekf(local_model, {});
  e::Ukf ukf(local_model, {});
  e::Vector state = e::Vector::Zero(STATE_SIZE);
  state(StateMemberVx) = 1.0;
  const e::Matrix covariance = e::Matrix::Identity(STATE_SIZE, STATE_SIZE);
  const e::Matrix noise = 0.01 * covariance;
  const e::Vector input;

  for (e::GaussianFilter* filter : {peer_filter.get(),
      static_cast<e::GaussianFilter*>(&ekf), static_cast<e::GaussianFilter*>(&ukf)}) {
    ASSERT_EQ(filter->initialize(state, covariance, 0), e::Status::ok);
    ASSERT_EQ(filter->predict(input, noise, 100'000'000, 100'000'000), e::Status::ok);
    EXPECT_EQ(filter->time(), 100'000'000u);
  }
  EXPECT_NEAR(ekf.state()(StateMemberX), 0.1, 1e-12);
  EXPECT_TRUE(peer_filter->state().isApprox(ukf.state(), 1e-10));
  EXPECT_TRUE(peer_filter->covariance().isApprox(ukf.covariance(), 1e-10));
}
}  // namespace rpp_localization::test
