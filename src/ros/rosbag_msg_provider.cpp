#include <rpp_localization/ros/rosbag_msg_provider.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace rpp_localization
{

  RosBagMsgProvider::RosBagMsgProvider(std::string bag_path, std::string storage_id)
    : _bag_path(bag_path),
      _storage_id(storage_id),
      _initialized(false)
  {
    rosbag2_storage::StorageOptions storage_options{};

    storage_options.uri = _bag_path;
    storage_options.storage_id = _storage_id;

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    _reader.open(storage_options, converter_options);


    _topics = _reader.get_all_topics_and_types();

    std::transform(_topics.begin(), _topics.end(), std::back_inserter(_topic_names),
              [&](const auto& topic){ return topic.name; });

    if (_reader.has_next())
    {
      _newest_msg = _reader.read_next();
      _start_time = _newest_msg->time_stamp;
      _initialized = true;
    }
  }

  RosBagMsgProvider::~RosBagMsgProvider()
  {

  }

  void RosBagMsgProvider::set_on_message_callback(std::function<bool(const SerializedMessage*, const TopicMetadata&, Serializator&)> callback)
  {
    _func = callback;
  }


  uint64_t RosBagMsgProvider::get_num_msgs() const
  {
    return _reader.get_metadata().message_count;
  }

  uint64_t RosBagMsgProvider::get_duration() const
  {
    return _reader.get_metadata().duration.count();
  }

  bool RosBagMsgProvider::has_next(rclcpp::Time future)
  {
    if (future == _zero_time)
    {
      return _newest_msg.get() != NULL;
    }
    return _newest_msg->time_stamp < _start_time + future.nanoseconds();
  }

  rcutils_time_point_value_t RosBagMsgProvider::get_start_time()
  {
    if (_initialized) return _start_time;
    return 0;
  }

  void RosBagMsgProvider::get_next(rclcpp::Time future)
  {
    if (_newest_msg == NULL) return;
    while (_newest_msg->time_stamp < future.nanoseconds())
    {
      rclcpp::SerializedMessage extracted_serialized_msg(*_newest_msg->serialized_data);
      std::vector<std::string>::iterator itr =
          std::find(_topic_names.begin(), _topic_names.end(), _newest_msg->topic_name);

      if (itr != _topic_names.end())
      {
        auto topic = _topics.at(std::distance(_topic_names.begin(), itr));
        _func(&extracted_serialized_msg, topic, _serializator);
      }
      if (_reader.has_next())
      {
        _newest_msg = _reader.read_next();
      }
      else
      {
        _newest_msg = nullptr; // set to null when finished
        break;
      }
    }
  }

} // rpp_localization
