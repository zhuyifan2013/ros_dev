/*
 * Copyright 2022 XTARK ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TARKBOT_ROBOT_H
#define TARKBOT_ROBOT_H

//C语言相关头文件
#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <termios.h>

//Boost库文件
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

//ROS相关头文件
#include <rclcpp/rclcpp.hpp>    
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/parameter_events_filter.hpp"

// #include <dynamic_reconfigure/server.h>
// #include <tarkbot_robot/robotConfig.h>
#include "tarkbot_robot/srv/light_set.hpp"

#define PI 3.1415926

// Robot communication frame headers
#define ID_CPR2ROS_DATA    0x10    // Data from lower computer to ROS
#define ID_ROS2CTR_VEL     0x50    // Velocity data from ROS to lower computer
#define ID_ROS2CTR_IMU     0x51    // IMU calibration command from ROS to lower computer
#define ID_ROS2CTR_RTY     0x5a    // Robot type parameter from ROS to lower computer
#define ID_ROS2CTR_LGT     0x52    // Light debug data from ROS to lower computer
#define ID_ROS2CTR_LST     0x53    // Light save data from ROS to lower computer
#define ID_ROS2CTR_BEEP    0x54    // Beep data from ROS to lower computer

#define LIGHT_M1    0x10    // Single color mode
#define LIGHT_M2    0x20    // Breathing mode

// IMU accelerometer range ±2g, corresponding data range ±32768
// Accelerometer raw data conversion to m/s^2 unit, 32768/2g=32768/19.6=1671.84
#define ACC_RATIO    (2*9.8/32768)

// IMU gyroscope range ±500°, corresponding data range ±32768
// Gyroscope raw data conversion to radians (rad) unit
#define GYRO_RATIO   ((500*PI/180)/32768)

// Robot data processing cycle, unit S
#define DATA_PERIOD   0.02f

typedef boost::shared_ptr<boost::asio::serial_port> serial_ptr;

// IMU data structure
struct ImuData
{
    float acc_x;
    float acc_y;
    float acc_z;    

    float gyro_x;
    float gyro_y;
    float gyro_z;
};

// IMU quaternion structure
struct ImuOrientationData
{
    float w;
    float x;
    float y;
    float z;    
};

// Robot velocity data structure
struct VelocityData
{
    float linear_x;
    float linear_y;
    float angular_z;    
};

// Robot position data structure
struct PoseData
{
    float pos_x;
    float pos_y;
    float angular_z;    
};

// Robot chassis drive class
class TarkbotRobot : public rclcpp::Node
{
public:
    TarkbotRobot();   // Constructor
    ~TarkbotRobot();  // Destructor

private:
    // Serial port operations
    bool openSerialPort();
    void closeSerialPort();

    // Serial port send data, TARK X-Protocol protocol
    void sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num);
    
    // Serial port multithreaded receive function
    void recvCallback();
    void recvDataHandle(uint8_t* buffer_data);

    // Velocity message callback function
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // Beep callback function
    void beep_callback(const std_msgs::msg::Int8::SharedPtr beep);
    // Light service callback function
    void light_ser_back(const std::shared_ptr<tarkbot_robot::srv::LightSet::Request> req,
                              std::shared_ptr<tarkbot_robot::srv::LightSet::Response> resp);

    // Publish topic messages
    void publishOdom();     // Publish odometry topic
    void publishImu();      // Publish IMU sensor topic
    void publishBatVol();   // Publish battery voltage topic

    // Publish odom TF transform
    void publishOdomTF();   

    // Calculate IMU quaternion
    void calculateImuQuaternion(struct ImuData imu_cel);

    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle_;

      // 参数回调函数
    // rcl_interfaces::msg::SetParametersResult on_parameter_change(const rclcpp::Parameter& parameters);

    void on_parameter_change(const rclcpp::Parameter& param);

    bool change_enable;
    // Serial port pointer
    std::string     serial_port_;      
    int             serial_port_baud_;
    serial_ptr      serial_ptr_;
    boost::system::error_code err_code_;
    boost::asio::io_service io_service_;

    // // Data definitions
    // ImuData imu_data_;    // IMU data
    // ImuOrientationData orient_data_;  // IMU quaternion attitude data
    // VelocityData vel_data_;    // Robot velocity
    // PoseData pos_data_;    // Robot position
    // float  bat_vol_data_;    // Robot battery voltage data

    //数据定义
    struct ImuData imu_data_;    //IMU数据
    struct ImuOrientationData orient_data_;  //IMU四元数姿态数据
    struct VelocityData vel_data_;    //机器人的速度
    struct PoseData pos_data_;    //机器人的位置
    float  bat_vol_data_;    //机器人电池电压数据

    // Light parameters
    int     rgb_m_; // Light mode
    int     rgb_s_; // Reserved
    int     rgb_t_;  // Reserved
    int     rgb_r_;  // Red light ratio
    int     rgb_g_;  // Green light ratio
    int     rgb_b_;  // Blue light ratio

    // Frame definitions
    std::string odom_frame_;    // Odometry
    std::string base_frame_;  // Robot
    std::string imu_frame_;     // IMU

    // Topic definitions
    std::string odom_topic_;    // Odometry
    std::string imu_topic_;     // IMU topic
    std::string bat_topic_;     // Battery topic
    std::string cmd_vel_topic_;     // Velocity command topic
    std::string beep_topic_;    // Beep topic
    
    // Robot type parameter
    std::string robot_type_send_;  

    // Message definitions
    nav_msgs::msg::Odometry odom_msgs_;  // Odometry publish message
    sensor_msgs::msg::Imu imu_msgs_;  // IMU publish message
    std_msgs::msg::Float32 bat_msgs_;  // Battery voltage publish message

    // Publisher definitions
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;  
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Subscriber definitions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr beep_sub_;

    // Service server
    rclcpp::Service<tarkbot_robot::srv::LightSet>::SharedPtr light_server_;
   
    // TF 坐标变换相关定义
    bool pub_odom_tf_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif
