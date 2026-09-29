#ifndef RPP_LOCALIZATION__FILTER_RESULT_HPP_
#define RPP_LOCALIZATION__FILTER_RESULT_HPP_


#include "rpp_localization/core/types.hpp"
#include <vector>

namespace rpp_localization
{

  struct FilterResult
  {
    std::vector<StateVector> states;
    std::vector<CovarianceMatrix> covariances;
    std::vector<double> timestamps;
  };


}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_RESULT_HPP_
