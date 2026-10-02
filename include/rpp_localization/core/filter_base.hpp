#pragma once

#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/model_base.hpp"

#include <ostream>
#include <stdexcept>

namespace rpp_localization
{

// Core contract for filters. The model is non-owning and must outlive its filter;
// the RPP composition root owns both objects.
class FilterBase
{
public:
  explicit FilterBase(const int state_dim)
  : _state_dim(state_dim),
    _debug(false),
    _debug_stream(nullptr),
    _model_as_base(nullptr)
  {
    if (state_dim <= 0)
    {
      throw std::invalid_argument("state_dim must be >0");
    }
  }

  virtual ~FilterBase() = default;

  virtual void correct(const Measurement& measurement) = 0;
  virtual void predict(TimestampNs reference_time, DurationNs delta) = 0;

  [[nodiscard]] ModelBase* get_model()
  {
    return _model_as_base;
  }

  [[nodiscard]] StateVector get_state() const
  {
    if (_model_as_base == nullptr)
    {
      throw std::logic_error("filter model is not initialized");
    }
    return _model_as_base->get_state();
  }

  [[nodiscard]] bool get_debug() const
  {
    return _debug;
  }

  void set_debug(const bool debug, std::ostream* output_stream)
  {
    if (debug && output_stream != nullptr)
    {
      _debug_stream = output_stream;
      _debug = true;
    }
    else
    {
      _debug = false;
    }

    if (_model_as_base != nullptr)
    {
      _model_as_base->set_debug(debug, output_stream);
    }
  }

protected:
  void set_model_base(ModelBase& model_base)
  {
    _model_as_base = &model_base;
  }

  int _state_dim;
  bool _debug;
  std::ostream* _debug_stream;
  ModelBase* _model_as_base;
};

}  // namespace rpp_localization
