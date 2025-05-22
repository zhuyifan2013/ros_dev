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

 #include "tarkbot_robot.h"

 /*
  * @功  能  主函数，ROS2初始化，调用构造函数初始化
  */
 int main(int argc, char** argv)
 {
     // ROS2初始化
     rclcpp::init(argc, argv);
 
     RCLCPP_INFO(rclcpp::get_logger("tarkbot_robot"), "Tarkbot Robot is Starting ......");
     
     // 创建节点并运行
     auto node = std::make_shared<TarkbotRobot>();

     RCLCPP_INFO(rclcpp::get_logger("tarkbot_robot"), "Node created");
 
     RCLCPP_INFO(rclcpp::get_logger("tarkbot_robot"), "Start to spin!!");
     rclcpp::spin(node);
     
     // 关闭ROS2
     rclcpp::shutdown();
     
     return 0;
 } 
 
 /*
  * @功  能  构造函数，机器人初始化
  */
 TarkbotRobot::TarkbotRobot() : Node("tarkbot_robot_node")
 {
     // ROS2参数处理方式
     this->declare_parameter("odom_frame", "odom");
     this->declare_parameter("base_frame", "base_link");
     this->declare_parameter("imu_frame", "imu_link");
     
     this->declare_parameter("odom_topic", "odom");
     this->declare_parameter("imu_topic", "imu");
     this->declare_parameter("battery_topic", "bat_vol");
     this->declare_parameter("cmd_vel_topic", "cmd_vel");
     this->declare_parameter("beep_topic", "beep");
     
     this->declare_parameter("robot_port", "/dev/ttyTHS1");
     this->declare_parameter("robot_port_baud", 230400);
     this->declare_parameter("pub_odom_tf", true);
     
     this->declare_parameter("robot_type_send", "r20_mec");
     
     // 获取参数
     odom_frame_ = this->get_parameter("odom_frame").as_string();
     base_frame_ = this->get_parameter("base_frame").as_string();
     imu_frame_ = this->get_parameter("imu_frame").as_string();
     
     odom_topic_ = this->get_parameter("odom_topic").as_string();
     imu_topic_ = this->get_parameter("imu_topic").as_string();
     bat_topic_ = this->get_parameter("battery_topic").as_string();
     cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
     beep_topic_ = this->get_parameter("beep_topic").as_string();
     
     serial_port_ = this->get_parameter("robot_port").as_string();
     serial_port_baud_ = this->get_parameter("robot_port_baud").as_int();
     pub_odom_tf_ = this->get_parameter("pub_odom_tf").as_bool();
     
     robot_type_send_ = this->get_parameter("robot_type_send").as_string();
 
     // ROS2发布者 - 使用shared_ptr和create_publisher方法
     odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);
     imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 50);
     bat_pub_ = this->create_publisher<std_msgs::msg::Float32>(bat_topic_, 10);
 
     // ROS2订阅者 - 使用create_subscription替代subscribe
     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
         cmd_vel_topic_, 100, std::bind(&TarkbotRobot::cmdVelCallback, this, std::placeholders::_1));
     
     beep_sub_ = this->create_subscription<std_msgs::msg::Int8>(
         beep_topic_, 10, std::bind(&TarkbotRobot::beep_callback, this, std::placeholders::_1));
     
     // ROS2服务 - 使用create_service替代advertiseService
     light_server_ = this->create_service<tarkbot_robot::srv::LightSet>(
         "Light_Server", std::bind(&TarkbotRobot::light_ser_back, this, 
                                 std::placeholders::_1, std::placeholders::_2));

        // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
 
     // 数据初始化
     memset(&pos_data_, 0, sizeof(pos_data_));
 
     // 提示信息，串口端口号和波特率
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot Set serial %s at %d baud", 
                 serial_port_.c_str(), serial_port_baud_);
 
     // 初始化串口
     if(openSerialPort())
     {
         try
         {
             boost::thread recvSerial_thread(boost::bind(&TarkbotRobot::recvCallback,this));
         }
         catch(...)
         {
             RCLCPP_INFO(this->get_logger(), 
                       "Tarkbot Robot can not open recvSerial_thread, Please check the serial port cable");
 
             // 关闭节点
             rclcpp::shutdown();
         }
     }
     else
     {
         // 关闭节点
         rclcpp::shutdown();
     } 
 
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot is Connected to OpenCTR board");
 
     // 声明参数
     this->declare_parameter<bool>("imu_calibrate", false);
     this->declare_parameter<bool>("light_calibrate", false);
     this->declare_parameter<int>("RGB_M", 0);
     this->declare_parameter<int>("RGB_S", 0);
     this->declare_parameter<int>("RGB_T", 0);
     this->declare_parameter<int>("RGB_R", 0);
     this->declare_parameter<int>("RGB_G", 0);
     this->declare_parameter<int>("RGB_B", 0);
 
     // 设置参数回调
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    //  parameter_callback_handle_ = parameter_event_handler_->add_parameter_callback(std::bind(&TarkbotRobot::on_parameter_change, this, std::placeholders::_1));

    auto cb =  [this](const rclcpp::Parameter& param) {
            this->on_parameter_change(param);
    };
    parameter_callback_handle_ = parameter_event_handler_->add_parameter_callback("yifan callback", cb);
        // parameter_callback_handle_ = parameter_event_handler_->add_parameter_callback("callback yifan", parameter_event_handler_);

     RCLCPP_INFO(this->get_logger(), "Ready to send data");
     // 向底层发送车型数据
     static uint8_t cartype_data[1];
     if(robot_type_send_ == "r20_mec")
         cartype_data[0] = 1;
     else if(robot_type_send_ == "r20_fwd")
         cartype_data[0] = 2;
     else if(robot_type_send_ == "r20_akm")
         cartype_data[0] = 3;
     else if(robot_type_send_ == "r20_twd")
         cartype_data[0] = 4;
     else if(robot_type_send_ == "r20_tak")
         cartype_data[0] = 5;
     else if(robot_type_send_ == "r20_omni")
         cartype_data[0] = 6;
     

     sendSerialPacket(cartype_data, 1, ID_ROS2CTR_RTY);

     RCLCPP_INFO(this->get_logger(), "Xtark Robot Set car type %s", robot_type_send_.c_str());
 
     // 提示信息，发布订阅的话题消息
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot setup publisher on odom [nav_msgs/msg/Odometry]");
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot setup publisher on imu [sensor_msgs/msg/Imu]");
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot setup publisher on bat_vol [float]");
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot setup subscriber on cmd_vel [geometry_msgs/msg/Twist]");
    
     // 机器人启动完成提示信息
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot initialization completed, is Running!");

         // 发送 static TF
     publishLaserTF();
     
     change_enable = false;
 }
 
 /*
  * @功  能  析构函数，对象生命周期结束时系统会调用这个函数
  */
 TarkbotRobot::~TarkbotRobot()
 { 
     static uint8_t vel_data[11];
     static uint8_t beep_data[1];
 
     // 发送静止指令
     vel_data[0] = 0;
     vel_data[1] = 0;
     vel_data[2] = 0;
     vel_data[3] = 0;
     vel_data[4] = 0;
     vel_data[5] = 0;
     sendSerialPacket(vel_data, 6, ID_ROS2CTR_VEL);
 
     // 数据转换
     beep_data[0] = 0;
 
     // 发送串口数据
     sendSerialPacket(beep_data, 1, ID_ROS2CTR_BEEP);
 
     // 关闭串口
     closeSerialPort();
 
     // 提示信息
     RCLCPP_INFO(this->get_logger(), "Tarkbot Robot shutting down."); 
 }
 
 /*
  * @功  能  初始化串口
  */
 bool TarkbotRobot::openSerialPort()
 {
     // 检查串口是否已经被打开
     if(serial_ptr_)
     {
         RCLCPP_INFO(this->get_logger(), "The SerialPort is already opened!");
         return false;
     }
 
     // 开打串口
     serial_ptr_ = serial_ptr(new boost::asio::serial_port(io_service_));
     serial_ptr_->open(serial_port_, err_code_);
 
     // 串口是否正常打开
     if(err_code_)
     {
         RCLCPP_INFO(this->get_logger(), "Open Port: %s Failed! Abort!", serial_port_.c_str());
         return false;
     }
 
     // 初始化串口参数
     serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(serial_port_baud_));
     serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
     serial_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
     serial_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
     serial_ptr_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
     
     return true;
 }
 
 /*
  * @功  能  关闭串口
  */
 void TarkbotRobot::closeSerialPort()
 {
     // 如果串口被打开，则关闭串口
     if(serial_ptr_)
     {
         serial_ptr_->cancel();
         serial_ptr_->close();
         serial_ptr_.reset();
     }
 
     // 停止io服务
     io_service_.stop();
     io_service_.reset();
 }
 
 // 协议解析变量
 uint8_t rx_con = 0;  // 接收计数器
 uint8_t rx_checksum; // 帧头部分校验和
 uint8_t rx_buf[60];  // 接收缓冲
 
 /*
  * @功  能  串口接收回调函数，接收塔克X-Protocol
  */
 void TarkbotRobot::recvCallback()
 {
     // 接收数据
     uint8_t res;
 
     while(rclcpp::ok())
     {
         // 读取串口数据
         boost::asio::read(*serial_ptr_.get(), boost::asio::buffer(&res, 1), err_code_);
 
         // 塔克X-Protocol协议解析数据
         if(rx_con < 3)    // =========接收帧头 + 长度
         {
             if(rx_con == 0)  // 接收帧头1 0xAA
             {
                 if(res == 0xAA)
                 {
                     rx_buf[0] = res;
                     rx_con = 1;                  
                 }
                 else
                 {
                     
                 }
             }
             else if(rx_con == 1) // 接收帧头2 0x55
             {
                 if(res == 0x55)
                 {
                     rx_buf[1] = res;
                     rx_con = 2;
                 }
                 else
                 {
                     rx_con = 0;                      
                 }               
             }
             else  // 接收数据长度
             {
                 rx_buf[2] = res;
                 rx_con = 3;
                 rx_checksum = (0xAA+0x55) + res;    // 计算校验和
             }
         }
         else    // =========接收数据
         {
             if(rx_con < (rx_buf[2]-1))
             {
                 rx_buf[rx_con] = res;
                 rx_con++;
                 rx_checksum = rx_checksum + res;                   
             }
             else    // 判断最后1位
             {
                 // 接收完成，恢复初始状态
                 rx_con = 0;                 
                 
                 // 数据校验
                 if(res == rx_checksum)  // 校验正确
                 {   
                     // 调用串口数据处理函数
                     recvDataHandle(rx_buf);
                 }   
             }
         }         
     }
 }
 
 /*
  * @功  能  串口接收数据处理
  */
 void TarkbotRobot::recvDataHandle(uint8_t* buffer_data)
 {
     // 机器人数据
     if(buffer_data[3] == ID_CPR2ROS_DATA)
     {
         // 解析IMU加速度数据
         imu_data_.acc_x = ((double)((int16_t)(buffer_data[4]*256+buffer_data[5]))*ACC_RATIO);
         imu_data_.acc_y = ((double)((int16_t)(buffer_data[6]*256+buffer_data[7]))*ACC_RATIO);
         imu_data_.acc_z = ((double)((int16_t)(buffer_data[8]*256+buffer_data[9]))*ACC_RATIO);
 
         // 解析IMU陀螺仪数据
         imu_data_.gyro_x = ((double)((int16_t)(buffer_data[10]*256+buffer_data[11]))*GYRO_RATIO);
         imu_data_.gyro_y = ((double)((int16_t)(buffer_data[12]*256+buffer_data[13]))*GYRO_RATIO);
         imu_data_.gyro_z = ((double)((int16_t)(buffer_data[14]*256+buffer_data[15]))*GYRO_RATIO);
 
         // 解析机器人速度
         vel_data_.linear_x = ((double)((int16_t)(buffer_data[16]*256+buffer_data[17]))/1000);
         vel_data_.linear_y = ((double)((int16_t)(buffer_data[18]*256+buffer_data[19]))/1000);
         vel_data_.angular_z = ((double)((int16_t)(buffer_data[20]*256+buffer_data[21]))/1000);
 
         // 解析电压值
         bat_vol_data_ = (double)(((buffer_data[22]<<8)+buffer_data[23]))/100;
 
         // 计算里程计数据
         pos_data_.pos_x += (vel_data_.linear_x*cos(pos_data_.angular_z) - vel_data_.linear_y*sin(pos_data_.angular_z)) * DATA_PERIOD; 
         pos_data_.pos_y += (vel_data_.linear_x*sin(pos_data_.angular_z) + vel_data_.linear_y*cos(pos_data_.angular_z)) * DATA_PERIOD; 
         pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD;   // 绕Z轴的角位移，单位：rad 
 
         // 计算IMU四元数数据
         calculateImuQuaternion(imu_data_);
 
         // 发布话题消息
         publishOdom();     // 发布里程计话题
         publishImu();      // 发布IMU传感器话题
         publishBatVol();   // 发布电池电压话题
 
         // 发布odom里程计TF坐标变换（odom --> base_link
         publishOdomTF();
     }
 }
 
 /*
  * @功  能  发布IMU话题消息
  */
 void TarkbotRobot::publishImu()
 {
     // 创建消息对象
     auto imu_msgs = std::make_unique<sensor_msgs::msg::Imu>();
     
     // 获取数据
     imu_msgs->header.stamp = this->now();
     imu_msgs->header.frame_id = imu_frame_;
     imu_msgs->angular_velocity.x = imu_data_.gyro_x;
     imu_msgs->angular_velocity.y = imu_data_.gyro_y;
     imu_msgs->angular_velocity.z = imu_data_.gyro_z;
     imu_msgs->linear_acceleration.x = imu_data_.acc_x;
     imu_msgs->linear_acceleration.y = imu_data_.acc_y;
     imu_msgs->linear_acceleration.z = imu_data_.acc_z;
     imu_msgs->orientation.x = orient_data_.x; 
     imu_msgs->orientation.y = orient_data_.y; 
     imu_msgs->orientation.z = orient_data_.z;
     imu_msgs->orientation.w = orient_data_.w;
 
     // 不使用姿态角度
     imu_msgs->orientation.x = 0; 
     imu_msgs->orientation.y = 0; 
 
     // 协方差矩阵
     imu_msgs->orientation_covariance = {  1e6,    0,    0,
                                            0,  1e6,    0,
                                            0,    0, 1e-6};
     imu_msgs->angular_velocity_covariance = { 1e6,    0,    0,
                                                0,  1e6,    0,
                                                0,    0, 1e-6};
     imu_msgs->linear_acceleration_covariance = {    0,    0,    0,
                                                    0,    0,    0,
                                                    0,    0,    0};
 
     // 发布
     imu_pub_->publish(std::move(imu_msgs));
 }
 
 /*
  * @功  能  发布odom里程计话题消息
  */
 void TarkbotRobot::publishOdom()
 {

    if (!odom_pub_) {
        RCLCPP_ERROR(this->get_logger(), "odom_pub_ is null!");
        return;
    }
     // 创建消息对象
     auto odom_msgs = std::make_unique<nav_msgs::msg::Odometry>();
     
     // 计算里程计四元数
     tf2::Quaternion odom_quat;
     odom_quat.setRPY(0, 0, pos_data_.angular_z);       
 
     // 获取数据
     odom_msgs->header.stamp = this->now();
     odom_msgs->header.frame_id = odom_frame_;
     odom_msgs->child_frame_id = base_frame_;
     odom_msgs->pose.pose.position.x = pos_data_.pos_x;
     odom_msgs->pose.pose.position.y = pos_data_.pos_y;
     odom_msgs->pose.pose.position.z = 0;  // 高度为0
     odom_msgs->pose.pose.orientation.x = odom_quat.getX();
     odom_msgs->pose.pose.orientation.y = odom_quat.getY();
     odom_msgs->pose.pose.orientation.z = odom_quat.getZ();
     odom_msgs->pose.pose.orientation.w = odom_quat.getW();
     odom_msgs->twist.twist.linear.x = vel_data_.linear_x;
     odom_msgs->twist.twist.linear.y = vel_data_.linear_y;
     odom_msgs->twist.twist.angular.z = vel_data_.angular_z;
 
     // 里程计协方差矩阵，用于robot_pose_ekf功能包，静止和运动使用不同的参数
     if(vel_data_.linear_x==0 && vel_data_.linear_y==0 && vel_data_.angular_z==0)
     {
         // 机器人静止时，IMU水平陀螺仪会存在零飘，编码器没有误差，编码器数据权重增加
         odom_msgs->pose.covariance = {  1e-9,    0,    0,    0,    0,    0, 
                                           0, 1e-3, 1e-9,    0,    0,    0,
                                           0,    0,  1e6,    0,    0,    0,
                                           0,    0,    0,  1e6,    0,    0,
                                           0,    0,    0,    0,  1e6,    0,
                                           0,    0,    0,    0,    0, 1e-9 };
 
         odom_msgs->twist.covariance = { 1e-9,    0,    0,    0,    0,    0, 
                                           0, 1e-3, 1e-9,    0,    0,    0,
                                           0,    0,  1e6,    0,    0,    0,
                                           0,    0,    0,  1e6,    0,    0,
                                           0,    0,    0,    0,  1e6,    0,
                                           0,    0,    0,    0,    0, 1e-9 };
     }
     else
     {
         // 机器人运动时，轮子滑动编码器误差增加，IMU陀螺仪数据更加准确，IMU数据权重增加
         odom_msgs->pose.covariance = {  1e-3,    0,    0,    0,    0,    0, 
                                           0, 1e-3,    0,    0,    0,    0,
                                           0,    0,  1e6,    0,    0,    0,
                                           0,    0,    0,  1e6,    0,    0,
                                           0,    0,    0,    0,  1e6,    0,
                                           0,    0,    0,    0,    0,  1e3 };
 
         odom_msgs->twist.covariance = { 1e-3,    0,    0,    0,    0,    0, 
                                           0, 1e-3,    0,    0,    0,    0,
                                           0,    0,  1e6,    0,    0,    0,
                                           0,    0,    0,  1e6,    0,    0,
                                           0,    0,    0,    0,  1e6,    0,
                                           0,    0,    0,    0,    0,  1e3 };
     }
 
     // 发布
     odom_pub_->publish(std::move(odom_msgs));
 } 
 /*
  * @功  能  发布电池电压话题消息
  */
 void TarkbotRobot::publishBatVol()
 {
     // 创建消息对象
     auto bat_msgs = std::make_unique<std_msgs::msg::Float32>();
     
     // 获取数据
     bat_msgs->data = bat_vol_data_;
 
     // 发布
     bat_pub_->publish(std::move(bat_msgs));
 }
 
 /*
  * @功  能  发布里程计到base_footprint的TF坐标变换
  */
 void TarkbotRobot::publishOdomTF()
 {
     // 发布里程计到footprint坐标变换
     if(pub_odom_tf_ == true)
     {
         // 计算里程计TF四元数
         tf2::Quaternion q;
         q.setRPY(0, 0, pos_data_.angular_z);
 
         // 创建TransformStamped消息
         geometry_msgs::msg::TransformStamped transform_stamped;
 
         // 填充数据
         transform_stamped.header.stamp = this->now();
         transform_stamped.header.frame_id = odom_frame_;
         transform_stamped.child_frame_id = base_frame_;
 
         transform_stamped.transform.translation.x = pos_data_.pos_x;
         transform_stamped.transform.translation.y = pos_data_.pos_y;
         transform_stamped.transform.translation.z = 0.0;
 
         transform_stamped.transform.rotation.x = q.x();
         transform_stamped.transform.rotation.y = q.y();
         transform_stamped.transform.rotation.z = q.z();
         transform_stamped.transform.rotation.w = q.w();

         // 发布TF坐标变换
         tf_broadcaster_->sendTransform(transform_stamped);
     }
 }

 void TarkbotRobot::publishLaserTF()
 {
     // 发布 base_link 到 laser 坐标变换
     static bool published = false; // 只发布一次
     if (!published)
     {
        RCLCPP_INFO(this->get_logger(), "1111");
         geometry_msgs::msg::TransformStamped laser_transform;
 
         // 填充时间戳和坐标系名字
         laser_transform.header.stamp = rclcpp::Time(0);
         laser_transform.header.frame_id = base_frame_;  // 通常是 "base_link"
         laser_transform.child_frame_id = "laser_link";  // 你自己定义的，比如 "laser"
 
         // 设定雷达在小车上的相对位置
         laser_transform.transform.translation.x = 0.1;   // 假设雷达前移 0.1 米
         laser_transform.transform.translation.y = 0.1;   // 不正中安装
         laser_transform.transform.translation.z = 0.12;   // 离地 0.12 米高
 
         // 雷达通常无旋转，或者绕 Z 轴转 90 度（看你的雷达安装方向）
         tf2::Quaternion q;
         q.setRPY(0, 0, 0);  // 无旋转
         laser_transform.transform.rotation.x = q.x();
         laser_transform.transform.rotation.y = q.y();
         laser_transform.transform.rotation.z = q.z();
         laser_transform.transform.rotation.w = q.w();
 
         // 发布静态TF
         tf_static_broadcaster_->sendTransform(laser_transform);
         RCLCPP_INFO(this->get_logger(), "Publishing static laser TF");
         published = true;
     }
 }
 
 /*
  * @功  能  cmd_vel 速度话题订阅回调函数，根据订阅的指令通过串口发指令控制下位机
  */
 void TarkbotRobot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
 {
     static uint8_t vel_data[11];
 
     // 数据转换
     vel_data[0] = (int16_t)(msg->linear.x * 1000) >> 8;
     vel_data[1] = (int16_t)(msg->linear.x * 1000);
     vel_data[2] = (int16_t)(msg->linear.y * 1000) >> 8;
     vel_data[3] = (int16_t)(msg->linear.y * 1000);
     vel_data[4] = (int16_t)(msg->angular.z * 1000) >> 8;
     vel_data[5] = (int16_t)(msg->angular.z * 1000);
 
     // 发送串口数据
     sendSerialPacket(vel_data, 6, ID_ROS2CTR_VEL);
 }
 
 /*
  * @功  能  灯光服务订阅回调函数，根据订阅的指令通过串口发指令控制下位机
  */
 void TarkbotRobot::light_ser_back(
     const std::shared_ptr<tarkbot_robot::srv::LightSet::Request> request,
     std::shared_ptr<tarkbot_robot::srv::LightSet::Response> response)
 {
     static uint8_t send_buff[11];
 
     // 数据转换
     send_buff[0] = (uint8_t)(request->rgb_m);
     send_buff[1] = (uint8_t)(request->rgb_s);
     send_buff[2] = (uint8_t)(request->rgb_t);
     send_buff[3] = (uint8_t)(request->rgb_r);
     send_buff[4] = (uint8_t)(request->rgb_g);
     send_buff[5] = (uint8_t)(request->rgb_b);
 
     if (send_buff[0] < 0 || send_buff[3] < 0 || send_buff[4] < 0 || send_buff[5] < 0 || send_buff[0] > 6)
     {
         RCLCPP_ERROR(this->get_logger(), "提交的数据异常:数据不可以为负数");
         response->result = "Failed to change light: Invalid data";
         return;
     }
 
     // 发送串口数据
     sendSerialPacket(send_buff, 6, ID_ROS2CTR_LGT);
     RCLCPP_INFO(this->get_logger(), "Light Control : M: %d S: %d T: %d R: %d, G: %d, B: %d", 
                 send_buff[0], send_buff[1], send_buff[2], send_buff[3], send_buff[4], send_buff[5]);
 
     // 如果没有异常，那么将结果赋值给 response
     response->result = "Success change light";
 }
 
 /*
  * @功  能  beep 蜂鸣器话题订阅回调函数，根据订阅的指令通过串口发指令控制下位机
  */
 void TarkbotRobot::beep_callback(const std_msgs::msg::Int8::SharedPtr beep)
 {
     static uint8_t beep_data[1];
 
     // 数据转换
     beep_data[0] = (uint8_t)(beep->data);
 
     // 发送串口数据
     sendSerialPacket(beep_data, 1, ID_ROS2CTR_BEEP);
 }
 
/*
 * @功  能  动态参数配置回调函数
 */

void TarkbotRobot::on_parameter_change(const rclcpp::Parameter& param)
 {
    //   rcl_interfaces::msg::SetParametersResult result;
    //  for (const auto &param : parameters)
    //  {
         if (param.get_name() == "imu_calibrate" && param.get_value<bool>())
         {
             RCLCPP_INFO(this->get_logger(), "Calibrating the IMU, Please hold the robot stationary for 5 seconds.");

             // 发送 IMU 校准指令
             uint8_t data[1] = {0x55};
             sendSerialPacket(data, 1, ID_ROS2CTR_IMU);

             rclcpp::sleep_for(std::chrono::milliseconds(500));

             // 校准复位
             this->set_parameter(rclcpp::Parameter("imu_calibrate", false));
         }

         if (param.get_name() == "light_calibrate" && param.get_value<bool>())
         {
             RCLCPP_INFO(this->get_logger(), "Calibrating the light.");

             // 发送灯光校准指令
             uint8_t data[1] = {0x55};
             sendSerialPacket(data, 1, ID_ROS2CTR_LST);

             rclcpp::sleep_for(std::chrono::milliseconds(500));

             // 校准复位
             this->set_parameter(rclcpp::Parameter("light_calibrate", false));
         }

         // 处理灯光设置
         if (change_enable)
         {
             if (param.get_name() == "RGB_M" || param.get_name() == "RGB_S" ||
                 param.get_name() == "RGB_T" || param.get_name() == "RGB_R" ||
                 param.get_name() == "RGB_G" || param.get_name() == "RGB_B")
             {
                 int rgb_m = this->get_parameter("RGB_M").get_value<int>();
                 int rgb_s = this->get_parameter("RGB_S").get_value<int>();
                 int rgb_t = this->get_parameter("RGB_T").get_value<int>();
                 int rgb_r = this->get_parameter("RGB_R").get_value<int>();
                 int rgb_g = this->get_parameter("RGB_G").get_value<int>();
                 int rgb_b = this->get_parameter("RGB_B").get_value<int>();

                 uint8_t data_light[6] = {
                     static_cast<uint8_t>(rgb_m),
                     static_cast<uint8_t>(rgb_s),
                     static_cast<uint8_t>(rgb_t),
                     static_cast<uint8_t>(rgb_r),
                     static_cast<uint8_t>(rgb_g),
                     static_cast<uint8_t>(rgb_b)};

                     sendSerialPacket(data_light, 6, ID_ROS2CTR_LGT);

                 RCLCPP_INFO(this->get_logger(),
                             "Tarkbot Robot Set rgb_m: %d, rgb_s: %d, rgb_t: %d, color_R: %d, color_G: %d, color_B: %d",
                             rgb_m, rgb_s, rgb_t, rgb_r, rgb_g, rgb_b);
             }
         }
    //  }

     if (!change_enable)
     {
         change_enable = true;
     }

    //  result.successful = true;
    //  result.reason = "OKK";
    //  return result;
 }

/*
 * @功  能  发送串口数据包，塔克X-Protocol协议
 * @参  数  *pbuf：发送数据指针
 *          len：发送数据长度个数，长度小于64字节
 *          num：帧号，帧编码
 * @返回值	 无
 */
 void TarkbotRobot::sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
 {
     uint8_t i, cnt;	
     uint8_t tx_checksum = 0; // 发送校验和
     uint8_t tx_buf[64];
     
     // 判断是否超出长度
     if(len <= 64)
     {
         // 获取数据
         tx_buf[0] = 0xAA;    // 帧头
         tx_buf[1] = 0x55;    //
         tx_buf[2] = len+5;   // 根据输出长度计算帧长度
         tx_buf[3] = num;     // 帧编码
         
         for(i=0; i<len; i++)
         {
             tx_buf[4+i] = *(pbuf+i);
         }
         
         // 计算校验和	
         cnt = 4+len;
         for(i=0; i<cnt; i++)
         {
             tx_checksum = tx_checksum + tx_buf[i];
         }
         tx_buf[i] = tx_checksum;
 
         // 计算帧长度
         cnt = len+5;
         
         // 发送数据 - 使用ROS2的串口接口
         boost::asio::write(*serial_ptr_.get(),boost::asio::buffer(tx_buf,cnt),err_code_);

     }
 }



/***************四元数计算**************************************************/
volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
 // quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;         
 // integral error terms scaled by Ki
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
volatile const float sampling_period  = DATA_PERIOD;

float invSqrt(float number);

/*
 * @功  能  计算IMU四元数
 */
void TarkbotRobot::calculateImuQuaternion(struct ImuData imu_cel)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float roll,pitch,yaw ;

    //首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = invSqrt(imu_cel.acc_x * imu_cel.acc_x + imu_cel.acc_y * imu_cel.acc_y + imu_cel.acc_z * imu_cel.acc_z);

    imu_cel.acc_x *= recipNorm;
    imu_cel.acc_y *= recipNorm;
    imu_cel.acc_z *= recipNorm;      

    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (imu_cel.acc_y * halfvz - imu_cel.acc_z * halfvy);
    halfey = (imu_cel.acc_z * halfvx - imu_cel.acc_x * halfvz);
    halfez = (imu_cel.acc_x * halfvy - imu_cel.acc_y * halfvx);

    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) 
    {
        integralFBx += twoKi * halfex * sampling_period;  // integral error scaled by Ki
        integralFBy += twoKi * halfey * sampling_period;
        integralFBz += twoKi * halfez * sampling_period;
        imu_cel.gyro_x += integralFBx;        // apply integral feedback
        imu_cel.gyro_y += integralFBy;
        imu_cel.gyro_z += integralFBz;
    }
    else 
    {
        integralFBx = 0.0f;       // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    // Apply proportional feedback
    imu_cel.gyro_x += twoKp * halfex;
    imu_cel.gyro_y += twoKp * halfey;
    imu_cel.gyro_z += twoKp * halfez;        

    // Integrate rate of change of quaternion
    imu_cel.gyro_x *= (0.5f * sampling_period);   // pre-multiply common factors
    imu_cel.gyro_y *= (0.5f * sampling_period);
    imu_cel.gyro_z *= (0.5f * sampling_period);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * imu_cel.gyro_x - qc * imu_cel.gyro_y - q3 * imu_cel.gyro_z);
    q1 += (qa * imu_cel.gyro_x + qc * imu_cel.gyro_z - q3 * imu_cel.gyro_y);
    q2 += (qa * imu_cel.gyro_y - qb * imu_cel.gyro_z + q3 * imu_cel.gyro_x);
    q3 += (qa * imu_cel.gyro_z + qb * imu_cel.gyro_y - qc * imu_cel.gyro_x); 

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //计算结果赋值到
    orient_data_.w = q0;
    orient_data_.x = q1;
    orient_data_.y = q2;
    orient_data_.z = q3;

    //计算欧拉角
    // roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	// pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	// yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    // ROS_INFO("IMU:%f  %f  %f ",roll,pitch,yaw);

}

/*
 * @功  能  平方根倒数 求四元数
 */
float invSqrt(float x)
{
    volatile long i;
    volatile float halfx, y;
    volatile const float f = 1.5F;

    halfx = x * 0.5F;
    y = x;
    i = * (( long * ) &y);
    
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( halfx * y * y ) );

    return y;
}