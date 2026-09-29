/*
 * SPDX-FileCopyrightText: (c) 2016, TNO IVS Helmond.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <functional>
#include <memory>
#include <string>

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/ros/listener.hpp"
#include "rpp_localization/srv/get_state.hpp"

namespace rpp_localization
{

class RppLocalizationListenerNode : public rclcpp::Node
{
public:
  RppLocalizationListenerNode()
  : rclcpp::Node("rpp_localization_listener_node")
  {
    service_ = this->create_service<rpp_localization::srv::GetState>(
      "get_state",
      std::bind(
        &RppLocalizationListenerNode::getStateCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  std::string getService()
  {
    return std::string(service_->get_service_name());
  }

  void setRosRppLocalizationListener(
    std::shared_ptr<rpp_localization::RosRppLocalizationListener> rll)
  {
    rll_ = rll;
  }

private:
  std::shared_ptr<RosRppLocalizationListener> rll_;
  rclcpp::Service<rpp_localization::srv::GetState>::SharedPtr service_;

  bool getStateCallback(
    const std::shared_ptr<rpp_localization::srv::GetState::Request> req,
    const std::shared_ptr<rpp_localization::srv::GetState::Response> res)
  {
    Eigen::VectorXd state(STATE_SIZE);
    Eigen::MatrixXd covariance(STATE_SIZE, STATE_SIZE);

    if (!rll_->get_state(req->time_stamp, req->frame_id, state, covariance)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Robot Localization Listener Node: Listener instance returned false at "
        "get_state call.");
      return false;
    }

    for (size_t i = 0; i < STATE_SIZE; i++) {
      res->state[i] = (*(state.data() + i));
    }

    for (size_t i = 0; i < STATE_SIZE * STATE_SIZE; i++) {
      res->covariance[i] = (*(covariance.data() + i));
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "Robot Localization Listener Node: Listener responded with state and "
      "covariance at the requested time.");
    return true;
  }
};

}  // namespace rpp_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto rlln = std::make_shared<
    rpp_localization::RppLocalizationListenerNode>();

  auto rll = std::make_shared<
    rpp_localization::RosRppLocalizationListener>(rlln);
  rlln->setRosRppLocalizationListener(rll);

  RCLCPP_INFO(
    rlln->get_logger(),
    "Robot Localization Listener Node: Ready to handle GetState requests at %s",
    rlln->getService().c_str());

  rclcpp::spin(rlln);

  return 0;
}
