#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rpp_localization/ros/time.hpp"


namespace rpp_localization
{

  class TfBufferWrapper
  {
    public:
      TfBufferWrapper(rclcpp::Clock::SharedPtr clock, bool online)
        : _online(online)
      {
        if (online)
        {
          _buffer.reset(new tf2_ros::Buffer(clock));
          _buffer_core = _buffer;
        }
        else
        {
          _buffer_core.reset(new tf2::BufferCore());
        }
      }


    void reset()
    {
      _buffer.reset();
      _buffer_core.reset();
    }

    void clear()
    {
      _buffer_core->clear(); // since it is the same object, core one clears all
    }

    bool set_transform(
      const geometry_msgs::msg::TransformStamped & transform,
      const std::string & authority, bool is_static = false)
    {
      return _buffer_core->setTransform(transform, authority, is_static);
    }
      //! @brief Method for safely obtaining transforms.
      //! @param[in] buffer - tf buffer object to use for looking up the transform
      //! @param[in] targetFrame - The target frame of the desired transform
      //! @param[in] sourceFrame - The source frame of the desired transform
      //! @param[in] time - The time at which we want the transform
      //! @param[in] timeout - How long to block before falling back to last transform
      //! @param[out] targetFrameTrans - The resulting transform object
      //! @param[in] silent - Whether or not to print transform warnings
      //! @return Sets the value of @p targetFrameTrans and returns true if
      //! successful, false otherwise.
      //!
      //! This method attempts to obtain a transform from the @p sourceFrame to the @p
      //! targetFrame at the specific @p time. If no transform is available at that
      //! time, it attempts to simply obtain the latest transform. If that still
      //! fails, then the method checks to see if the transform is going from a given
      //! frame_id to itself. If any of these checks succeed, the method sets the
      //! value of @p targetFrameTrans and returns true, otherwise it returns false.
      //!
      bool lookup_transform_safe(
        const std::string & target_frame,
        const std::string & source_frame,
        const rclcpp::Time & time,
        const rclcpp::Duration & duration,
        tf2::Transform & target_frame_trans,
        const bool silent=false)
      {
        bool retVal = true;
        tf2::TimePoint time_tf = tf2::timeFromSec(ros::to_seconds(time));


          tf2::Duration duration_tf =
            tf2::durationFromSec(ros::to_seconds(duration));

        // First try to transform the data at the requested time
        try
        {
          get_transform(target_frame, source_frame, time_tf, duration_tf, target_frame_trans);
        }
        catch (tf2::TransformException & ex)
        {
          // The issue might be that the transforms that are available are not close
          // enough temporally to be used. In that case, just use the latest available
          // transform and warn the user.
          try
          {
            get_transform(target_frame, source_frame, tf2::TimePointZero, duration_tf, target_frame_trans);

            if (!silent)
            {
              // ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << source_frame <<
              // " to " << target_frame <<
              //                              " was unavailable for the time
              //                              requested. Using latest instead.\n");
            }
          }
          catch (tf2::TransformException & ex)
          {
            if (!silent)
            {
              // ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " <<
              // source_frame <<
              //                              " to " << target_frame << ". Error was "
              //                              << ex.what() << "\n");
            }
            retVal = false;
          }
        }


        // Transforming from a frame id to itself can fail when the tf tree isn't
        // being broadcast (e.g., for some bag files). This is the only failure that
        // would throw an exception, so check for this situation before giving up.
        if (!retVal) {
          if (target_frame == source_frame) {
            target_frame_trans.setIdentity();
            retVal = true;
          }
        }

        return retVal;
      }

      //! @brief Method for safely obtaining transforms.
      //! @param[in] buffer - tf buffer object to use for looking up the transform
      //! @param[in] targetFrame - The target frame of the desired transform
      //! @param[in] sourceFrame - The source frame of the desired transform
      //! @param[in] time - The time at which we want the transform
      //! @param[out] targetFrameTrans - The resulting transform object
      //! @param[in] silent - Whether or not to print transform warnings
      //! @return Sets the value of @p targetFrameTrans and returns true if
      //! successful, false otherwise.
      //!
      //! This method attempts to obtain a transform from the @p sourceFrame to the @p
      //! targetFrame at the specific @p time. If no transform is available at that
      //! time, it attempts to simply obtain the latest transform. If that still
      //! fails, then the method checks to see if the transform is going from a given
      //! frame_id to itself. If any of these checks succeed, the method sets the
      //! value of @p targetFrameTrans and returns true, otherwise it returns false.
      //!
      bool lookup_transform_safe(
        const std::string & target_frame,
        const std::string & source_frame,
        const rclcpp::Time & time,
        tf2::Transform & target_frame_trans,
        const bool silent=false)
      {
        return lookup_transform_safe(
          target_frame, source_frame, time,
          rclcpp::Duration(0, 0u), target_frame_trans, silent);
      }

      geometry_msgs::msg::TransformStamped
      lookup_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        const tf2::TimePoint & time_tf)
      {
        return _buffer_core->lookupTransform(
          target_frame, source_frame, time_tf);
      }

      tf2::BufferCore& get_buffer()
      {
          return *_buffer_core;
      }

    private:
      void get_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        const tf2::TimePoint & time_tf,
        const tf2::Duration & duration_tf,
        tf2::Transform & target_frame_trans
      )
      {
        if (_online)
        {
          geometry_msgs::msg::TransformStamped stamped = _buffer->lookupTransform(
            target_frame, source_frame, time_tf, duration_tf);
          tf2::fromMsg(stamped.transform, target_frame_trans);
        }
        else
        {
            geometry_msgs::msg::TransformStamped stamped = _buffer_core->lookupTransform(
            target_frame, source_frame, time_tf);
            tf2::fromMsg(stamped.transform, target_frame_trans);
        }
      }

      std::shared_ptr<tf2::BufferCore> _buffer_core;
      std::shared_ptr<tf2_ros::Buffer> _buffer;
      bool _online;


  };

}