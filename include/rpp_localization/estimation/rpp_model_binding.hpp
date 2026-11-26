#pragma once

#include "rpp_localization/estimation/rpp_support.hpp"

#include <rpp_plugin_types/rpp_interfaces/Model.hpp>
#include <rpp_schema/rpp_interfaces/ModelDesc.hpp>
#include <capnp_gen/rpp_interfaces/model/constants.capnp.h>

namespace rpp_localization::estimation::shared
{
// Prepared translation between the numerical core and the generated Model API.
// No live Model pointer is retained beyond an owner's active-component guard.
class ModelBinding final : public ProcessModel
{
public:
  class Call
  {
  public:
    Call(ModelBinding& binding, rpp_interfaces::Model& model) : binding_(binding)
    {binding_.backend_ = &model; binding_.last_code = codes::CODE_OK;}
    ~Call() {binding_.backend_ = nullptr;}
    Call(const Call&) = delete;
    Call& operator=(const Call&) = delete;
  private:
    ModelBinding& binding_;
  };

  explicit ModelBinding(const rpp_schema::rpp_interfaces::ModelDesc_Native& description,
    const NativeStateObservation* projection = nullptr)
  : state_layout(description.state.fields), input_layout(description.inputs.fields),
    prediction_layout(description.prediction.fields), observation_layout(description.observations.fields),
    features(description.features), space_(vector_space(state_layout)),
    prediction_route_(rpp_interfaces::field_route(state_layout, prediction_layout))
  {
    if (prediction_layout.size() == 0 || !(features & codes::MODEL_PREDICT)) {
      throw std::invalid_argument("Estimator model must predict a nonempty subset of its state");
    }
    state_packet_.resize(state_layout.size());
    input_packet_.resize(input_layout.size());
    if (projection) {
      observation_projection_ = projection->observation_state_indices();
      if (observation_projection_.size() != observation_layout.size()) {
        throw std::invalid_argument("Native state observation width does not match the model description");
      }
    }
  }

  const VectorSpace& state_space() const override {return space_;}
  Eigen::Index input_size() const override {return static_cast<Eigen::Index>(input_layout.size());}
  const std::vector<Eigen::Index>& observation_projection() const {return observation_projection_;}

  bool predict(VectorIn state, VectorIn input, TimeNs time, TimeNs dt,
    TimeNs input_dt, VectorOut next) const override
  {
    if (!pack(state, input)) {return false;}
    auto [values, status] = backend_->predict(state_packet_, input_packet_, time, dt, input_dt);
    if (!result(status, values, prediction_layout.size())) {return false;}
    next = state;
    for (std::size_t index = 0; index < prediction_route_.size(); ++index) {
      next(static_cast<Eigen::Index>(prediction_route_[index])) = values[index];
    }
    return true;
  }

  bool linearize(VectorIn state, VectorIn input, TimeNs time, TimeNs dt,
    TimeNs input_dt, MatrixOut a) const override
  {
    if (!pack(state, input)) {return false;}
    auto [values, b, status] = backend_->linearize(state_packet_, input_packet_, time, dt, input_dt);
    if (!result(status, values, prediction_layout.size() * state_layout.size()) ||
      !result(codes::CODE_OK, b, prediction_layout.size() * input_layout.size())) {return false;}
    a.setIdentity();
    for (std::size_t row = 0; row < prediction_route_.size(); ++row) {
      for (std::size_t column = 0; column < state_layout.size(); ++column) {
        a(static_cast<Eigen::Index>(prediction_route_[row]), static_cast<Eigen::Index>(column)) =
          values[row * state_layout.size() + column];
      }
    }
    return true;
  }

  bool observe(VectorIn state, VectorIn input, TimeNs time, TimeNs input_dt,
    const std::vector<std::size_t>& route, VectorOut values) const
  {
    if (!pack(state, input)) {return false;}
    auto [full, status] = backend_->observe(state_packet_, input_packet_, time, input_dt);
    if (!result(status, full, observation_layout.size())) {return false;}
    for (std::size_t index = 0; index < route.size(); ++index) {
      values(static_cast<Eigen::Index>(index)) = full[route[index]];
    }
    return true;
  }

  bool observation_jacobian(VectorIn state, VectorIn input, TimeNs time, TimeNs input_dt,
    const std::vector<std::size_t>& route, MatrixOut h) const
  {
    if (!pack(state, input)) {return false;}
    auto [full, b, status] = backend_->observationJacobian(state_packet_, input_packet_, time, input_dt);
    if (!result(status, full, observation_layout.size() * state_layout.size()) ||
      !result(codes::CODE_OK, b, observation_layout.size() * input_layout.size())) {return false;}
    for (std::size_t row = 0; row < route.size(); ++row) {
      for (std::size_t column = 0; column < state_layout.size(); ++column) {
        h(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          full[route[row] * state_layout.size() + column];
      }
    }
    return true;
  }

  rpp_interfaces::FieldLayout state_layout, input_layout, prediction_layout, observation_layout;
  std::uint8_t features;
  mutable std::uint8_t last_code{codes::CODE_OK};

private:
  bool pack(VectorIn state, VectorIn input) const
  {
    if (!backend_) {last_code = codes::CODE_NOT_READY; return false;}
    write_vector(state, state_packet_);
    write_vector(input, input_packet_);
    return true;
  }
  bool result(std::uint8_t code, rpp::ListConst<double> values, std::size_t expected) const
  {
    if (code != codes::CODE_OK && code != codes::CODE_LIMITED) {last_code = code; return false;}
    if (values.size() != expected) {last_code = codes::CODE_INCOMPATIBLE; return false;}
    for (std::size_t index = 0; index < expected; ++index) {
      if (!std::isfinite(values[index])) {last_code = codes::CODE_INFEASIBLE; return false;}
    }
    if (code == codes::CODE_LIMITED) {last_code = code;}
    return true;
  }

  VectorSpace space_;
  std::vector<std::size_t> prediction_route_;
  std::vector<Eigen::Index> observation_projection_;
  mutable rpp::List<double> state_packet_, input_packet_;
  rpp_interfaces::Model* backend_{nullptr};
};

class ObservationBindingModel final : public ObservationModel
{
public:
  ObservationBindingModel(const ModelBinding& model, rpp_interfaces::FieldLayout layout)
  : layout(std::move(layout)), model_(model), space_(vector_space(this->layout)),
    route_(rpp_interfaces::field_route(model.observation_layout, this->layout)),
    covariance(this->layout.size(), this->layout.size()),
    measurement(this->layout.size()),
    validator(std::make_unique<CovarianceValidator>(static_cast<Eigen::Index>(this->layout.size())))
  {
    if (!model.observation_projection().empty()) {
      projection_.reserve(route_.size());
      for (const auto index : route_) {projection_.push_back(model.observation_projection()[index]);}
    }
  }

  Eigen::Index state_size() const override {return model_.state_space().size();}
  Eigen::Index input_size() const override {return model_.input_size();}
  const VectorSpace& observation_space() const override {return space_;}
  const std::vector<Eigen::Index>* state_projection() const noexcept override
  {return projection_.empty() ? nullptr : &projection_;}
  bool observe(VectorIn state, VectorIn input, TimeNs time, TimeNs input_dt,
    VectorOut values) const override
  {return model_.observe(state, input, time, input_dt, route_, values);}
  bool jacobian(VectorIn state, VectorIn input, TimeNs time, TimeNs input_dt,
    MatrixOut h) const override
  {return model_.observation_jacobian(state, input, time, input_dt, route_, h);}

  rpp_interfaces::FieldLayout layout;
private:
  const ModelBinding& model_;
  VectorSpace space_;
  std::vector<std::size_t> route_;
  std::vector<Eigen::Index> projection_;
public:
  Matrix covariance;
  Vector measurement;
  std::unique_ptr<CovarianceValidatorBase> validator;
};
}  // namespace rpp_localization::estimation::shared
