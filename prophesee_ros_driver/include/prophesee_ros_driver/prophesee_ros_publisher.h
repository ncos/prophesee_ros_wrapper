/*******************************************************************
 * File : prophesee_ros_publisher.h                                *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef PROPHESEE_ROS_PUBLISHER_H_
#define PROPHESEE_ROS_PUBLISHER_H_

#include <sensor_msgs/CameraInfo.h>

#include <prophesee_driver.h>
#include <mutex>

#include "log_tone_mapper.h"


template <class T>
class SwitchQueue {
private:
    std::mutex mtx;

    std::vector<T> q[2];
    int q_id = 0;

    std::vector<T> *q_in;
    std::vector<T> *q_out;

    void set_ptrs() {
        q_in = &(q[(q_id + 0) % 2]);
        q_out = &(q[(q_id + 1) % 2]);
    }

public:

    SwitchQueue() {
        set_ptrs();
        q_in->reserve(1000000);
        q_out->reserve(1000000);
    }

    void push(T &d) {
        mtx.lock();
        q_in->push_back(d);
        if (q_in->size() == q_in->capacity())
            ROS_WARN("Vector capacity maxed out! %d", q_in->size());
        mtx.unlock();
    }

    auto begin() {return q_out->begin(); }
    auto end()   {return q_out->end(); }
    auto size()  {return q_out->size(); }
    void clear() {q_out->clear(); }

    void swap() {
        mtx.lock();
        q_id = (q_id + 1) % 2;
        set_ptrs();
        mtx.unlock();
    }
};



/// \brief Main class for ROS publisher
///
/// Publishes data from Prophesee sensor to ROS topics
class PropheseeWrapperPublisher {
public:
    /// \brief Constructor
    PropheseeWrapperPublisher();

    /// \brief Destructor
    ~PropheseeWrapperPublisher();

    /// \brief Starts the camera and starts publishing data
    void startPublishing();

private:

    /// \brief Opens the camera
    bool openCamera();

    /// \brief Publishes CD events
    void publishCDEvents();

    /// \brief Publishes gray-level frames
    void publishGrayLevels();

    /// \brief Publishes IMU events
    void publishIMUEvents();

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Publisher for camera info
    ros::Publisher pub_info_;

    /// \brief Publisher for CD events
    ros::Publisher pub_cd_events_;

    /// \brief Publisher for gray-level frame
    ros::Publisher pub_gl_frame_;

    /// \brief Publisher for IMU events
    ros::Publisher pub_imu_events_;

    /// \brief Instance of Camera class
    ///
    /// Used to access data from a camera
    Prophesee::Camera camera_;

    /// \brief Instance of LogToneMapper class
    ///
    /// Used to reconstract gray-levels from CD and EM data and apply tone mapping
    LogToneMapper tone_mapper_;

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg_;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_file_;

    /// \brief Camera name in string format
    std::string camera_name_;

    /// \brief Camera serial number, to open a specific camera
    std::string camera_serial_;

    /// \brief Camera string time
    ros::Time start_timestamp_;

    /// \brief Maximum events rate, in kEv/s
    int max_event_rate_;

    /// \brief Grey-level rate, in fps
    int graylevel_rate_;

    /// \brief If showing CD events
    bool publish_cd_;

    /// \brief If showing gray-level frames
    bool publish_graylevels_;

    /// \brief If showing IMU events
    bool publish_imu_;

    static constexpr double GRAVITY = 9.81; /** Mean gravity value at Earth surface [m/s^2] **/

    int max_events = 10000;
    double max_time = 0.033;
    double previous_ts = 0.0;

    SwitchQueue<prophesee_event_msgs::Event> event_fifo;
};

#endif /* PROPHESEE_ROS_PUBLISHER_H_ */
