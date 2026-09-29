#ifndef RPP_LOCALIZATION__FILTER_RESULT_HPP_
#define RPP_LOCALIZATION__FILTER_RESULT_HPP_


#include <Eigen/Dense>
#include <vector>

namespace rpp_localization
{

  struct FilterResult
  {
    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::MatrixXd> covariances;
    std::vector<double> timestamps;
  };


}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_RESULT_HPP_
