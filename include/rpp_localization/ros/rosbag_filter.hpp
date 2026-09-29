# pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/ros/ros_filter_types.hpp"
#include "rpp_localization/ros/rosbag_msg_provider.hpp"
#include "rpp_localization/ros/ros_filter_base.hpp"
#include "rpp_localization/ros/ros_filter_utilities.hpp"
#include "rpp_localization/core/filter_result.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace rpp_localization;

namespace rpp_localization
{
template <typename T>
class RosBagFilter : public RosFilterBase<T>
{

  public:

    RosBagFilter(const rclcpp::NodeOptions& options, RosBagMsgProvider& provider, double frequency);
    ~RosBagFilter();
    bool on_new_msg(const SerializedMessage* msg, const TopicMetadata& topic, Serializator& serializator);
    FilterResult filter();
    void init();


  private:
    void load_params();
    void load_static_transforms();
    void update();
    RosBagMsgProvider& _provider;
    double _frequency;
    double _update_time;
    rclcpp::Time _total_time;
    std::string _filter_param_namespace;
    std::string _static_transform_param_namespace;
    std::map<std::string, CallbackData> pose_callback_data;
    std::map<std::string, CallbackData> twist_callback_data;
    std::map<std::string, CallbackData> acc_callback_data;

};
} // rpp_localization