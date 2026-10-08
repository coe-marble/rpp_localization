#include "gtest/gtest.h"
#include "rpp_localization/core/geodetic.hpp"

namespace rpp_localization
{
namespace
{

using geodetic::Geodetic;

TEST(GeodeticTest, datum_maps_to_the_origin)
{
  const Geodetic datum{45.8007257, 15.9721655, 120.0};

  EXPECT_LT(geodetic::to_enu(datum, datum).norm(), 1e-9);
}

TEST(GeodeticTest, ecef_of_the_equator_and_prime_meridian_is_the_semi_major_axis)
{
  const auto ecef = geodetic::to_ecef(Geodetic{0.0, 0.0, 0.0});

  EXPECT_NEAR(ecef.x(), geodetic::k_wgs84_semi_major_axis, 1e-6);
  EXPECT_NEAR(ecef.y(), 0.0, 1e-6);
  EXPECT_NEAR(ecef.z(), 0.0, 1e-6);
}

TEST(GeodeticTest, altitude_difference_is_the_up_component)
{
  const Geodetic datum{45.8, 15.97, 100.0};

  const auto enu = geodetic::to_enu(datum, Geodetic{45.8, 15.97, 112.5});

  EXPECT_NEAR(enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(enu.y(), 0.0, 1e-6);
  EXPECT_NEAR(enu.z(), 12.5, 1e-6);
}

TEST(GeodeticTest, small_offsets_map_to_east_and_north_metres)
{
  const Geodetic datum{45.8, 15.97, 0.0};
  // One arc second of latitude is about 30.87 m at this latitude; one arc
  // second of longitude is that scaled by roughly the cosine of latitude.
  const double arc_second = 1.0 / 3600.0;

  const auto north = geodetic::to_enu(datum, Geodetic{45.8 + arc_second, 15.97, 0.0});
  const auto east = geodetic::to_enu(datum, Geodetic{45.8, 15.97 + arc_second, 0.0});

  EXPECT_NEAR(north.y(), 30.87, 0.02);
  EXPECT_NEAR(north.x(), 0.0, 1e-3);
  EXPECT_NEAR(east.x(), 21.60, 0.02);
  EXPECT_NEAR(east.y(), 0.0, 1e-3);
}

TEST(GeodeticTest, positions_south_and_west_are_negative)
{
  const Geodetic datum{45.8, 15.97, 0.0};

  const auto enu = geodetic::to_enu(datum, Geodetic{45.799, 15.969, 0.0});

  EXPECT_LT(enu.x(), 0.0);
  EXPECT_LT(enu.y(), 0.0);
}

}  // namespace
}  // namespace rpp_localization
