#pragma once

#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/model_base.hpp"

#include <memory>
#include <ostream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace rpp_localization
{

// Compatibility base for legacy filters. The model pointer is non-owning;
// model ownership remains with each concrete filter.
class FilterBase
{
public:
  explicit FilterBase(const int state_dim)
  : _debug(false),
    _debug_stream(nullptr),
    _state_dim(state_dim),
    _model_as_base(nullptr)
  {
    if (state_dim <= 0)
    {
      throw std::invalid_argument("state_dim must be >0");
    }
  }

  virtual ~FilterBase() = default;

  virtual void init(std::shared_ptr<rclcpp::Node> node) = 0;
  virtual void correct(const Measurement& measurement) = 0;
  virtual void predict(
    const rclcpp::Time& reference_time,
    const rclcpp::Duration& delta) = 0;

  [[nodiscard]] ModelBase* get_model()
  {
    return _model_as_base;
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
  std::shared_ptr<rclcpp::Node> _node;
  ModelBase* _model_as_base;
};

}  // namespace rpp_localization
