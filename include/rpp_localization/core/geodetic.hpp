#ifndef RPP_LOCALIZATION__CORE__GEODETIC_HPP_
#define RPP_LOCALIZATION__CORE__GEODETIC_HPP_

#include <cmath>

#include "Eigen/Dense"

namespace rpp_localization
{
namespace geodetic
{

inline constexpr double k_wgs84_semi_major_axis = 6378137.0;
inline constexpr double k_wgs84_eccentricity_squared = 6.6943799901413165e-3;
inline constexpr double k_degrees_to_radians = 3.14159265358979323846 / 180.0;

/// A WGS-84 position in degrees and metres above the ellipsoid.
struct Geodetic
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude{0.0};
};

/// Earth-centred, earth-fixed position of a geodetic position, in metres.
inline Eigen::Vector3d to_ecef(const Geodetic& position)
{
  const double latitude = position.latitude_deg * k_degrees_to_radians;
  const double longitude = position.longitude_deg * k_degrees_to_radians;
  const double sin_latitude = std::sin(latitude);
  const double prime_vertical_radius = k_wgs84_semi_major_axis /
    std::sqrt(1.0 - k_wgs84_eccentricity_squared * sin_latitude * sin_latitude);
  return Eigen::Vector3d(
    (prime_vertical_radius + position.altitude) * std::cos(latitude) * std::cos(longitude),
    (prime_vertical_radius + position.altitude) * std::cos(latitude) * std::sin(longitude),
    (prime_vertical_radius * (1.0 - k_wgs84_eccentricity_squared) + position.altitude) *
    sin_latitude);
}

/// East, north, up position of a geodetic position relative to a datum.
inline Eigen::Vector3d to_enu(const Geodetic& datum, const Geodetic& position)
{
  const double latitude = datum.latitude_deg * k_degrees_to_radians;
  const double longitude = datum.longitude_deg * k_degrees_to_radians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);

  Eigen::Matrix3d ecef_to_enu;
  ecef_to_enu <<
    -sin_longitude, cos_longitude, 0.0,
    -sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude,
    cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude;
  return ecef_to_enu * (to_ecef(position) - to_ecef(datum));
}

}  // namespace geodetic
}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__CORE__GEODETIC_HPP_
