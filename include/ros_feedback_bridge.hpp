// Created by Indraneel on 3/02/21

#ifndef ROS_FEEDBACK_BRIDGE_HPP
#define ROS_FEEDBACK_BRIDGE_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include "robosar.pb.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>
#include "odom_node.hpp"
#include <nav_msgs/Odometry.h>
#include <mutex>
#include "easylogging++.h"
#include <ros/package.h>
class ROSFeedbackBridge
{

public:
    ROSFeedbackBridge(std::string robot_id_, ros::NodeHandle nh,const int fb_freq_):
        nh_(nh),node_alive_(true),odom_freq_hz(fb_freq_*2),khepera_frame(robot_id_+"/base_link"),rid(robot_id_),
        odom_data_pub_euler(nh_.advertise<nav_msgs::Odometry>("odom_data_euler", 100)),odom_data_pub_quat(nh_.advertise<nav_msgs::Odometry>("odom_data_quat", 100)),
        odom_node_(rid, odom_data_pub_euler, odom_data_pub_quat)
    {

        // Create ROS nodes for this agent
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("feedback/IMU", 1, true);
        lrf_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("feedback/scan", 1, true);
        agent_system_status_publisher_ = nh_.advertise<std_msgs::UInt32>("debug/status", 1, true);
        agent_system_msg_publisher_ = nh_.advertise<std_msgs::String>("debug/msg", 1, true);

        //// Start Odom node
        odom_delay_ms = (int)((1.0/(double)(odom_freq_hz))*1000.0);
        odom_thread_ = std::thread(&ROSFeedbackBridge::runOdometry, this);

        std::string package_path = ros::package::getPath("robosar_agent_bringup");

        // Create log file for this agent
        el::Configurations agentLogConf;
        agentLogConf.setToDefault();
        // Values are always std::string
        
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::Format, "%datetime %level %msg");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::ToFile,"true");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::ToStandardOutput,"false");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::Filename,package_path+"/logs/"+robot_id_+".txt");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::MaxLogFileSize,"1000000");
        // Create new logger
        logger = el::Loggers::getLogger(robot_id_);
        // default logger uses default configurations
        el::Loggers::reconfigureLogger(robot_id_, agentLogConf);

        logger->info("*************************************************************************");
        logger->info("All hail lord gupta");
        logger->info("*************************************************************************");
        
    }

    ~ROSFeedbackBridge()
    {
        ROS_INFO("Killing feedback bridge");
        node_alive_ = false;
        odom_thread_.join();
        el::Loggers::unregisterLogger(rid);
    }

    void unpack_feedback_message(robosar_fms::SensorData* feedback) {

        ROS_DEBUG("Unpacking message");

        //IMU
        sensor_msgs::Imu imu_msg;
        
        imu_msg.header.frame_id = khepera_frame;
        imu_msg.header.seq = feedback->seq_id();
        imu_msg.header.stamp = ros::Time::now();

        //@indraneel convert to m/s^2
        imu_msg.linear_acceleration.x = feedback->accel_data().acc_x();
        imu_msg.linear_acceleration.y = feedback->accel_data().acc_y();
        imu_msg.linear_acceleration.z = feedback->accel_data().acc_z();

        imu_msg.angular_velocity.x = angles::from_degrees(feedback->gyro_data().gyro_x());
        imu_msg.angular_velocity.y = angles::from_degrees(feedback->gyro_data().gyro_y());
        imu_msg.angular_velocity.z = angles::from_degrees(feedback->gyro_data().gyro_z());
        
        imu_publisher_.publish(imu_msg);

        logger->info("IMU seq :%v, accel_x :%v, accel_y :%v, accel_z :%v, ang_x :%v,  ang_y :%v,  ang_z :%v",imu_msg.header.seq,
                                                                                                            imu_msg.linear_acceleration.x,
                                                                                                            imu_msg.linear_acceleration.y,
                                                                                                            imu_msg.linear_acceleration.z,
                                                                                                            imu_msg.angular_velocity.x,
                                                                                                            imu_msg.angular_velocity.y,
                                                                                                            imu_msg.angular_velocity.z);
        // encoder callback
        int pos_left, pos_right;
        {
            std::lock_guard<std::mutex> guard(mtx);
            pos_left = feedback->count_data().left();
            pos_right = feedback->count_data().right();
            odom_node_.update_encoders(pos_left, pos_right);
        }
        logger->info("Left ticks: %v", pos_left);
        logger->info("Right ticks: %v", pos_right);

        // LaserScan
        sensor_msgs::LaserScan lrf_msg;

        lrf_msg.header.frame_id = khepera_frame;
        lrf_msg.header.stamp = ros::Time::now();
        lrf_msg.header.seq = feedback->seq_id();
        // verified from URG-04LX-UG01 specification datasheet
        lrf_msg.angle_min = -2.0923497676849365 + 0.006135923322290182;
        lrf_msg.angle_max = 2.0923497676849365;
        lrf_msg.angle_increment = 0.006135923322290182;
        lrf_msg.time_increment = 9.765627328306437e-05;
        lrf_msg.scan_time = 0.10000000149011612;
        lrf_msg.range_min = 0.019999999552965164;
        lrf_msg.range_max = 5.599999904632568;
        robosar_fms::LaserScanner lrf_feedback = feedback->lrf_data();
        //ROS_INFO("Lrf data size : %d\n",lrf_feedback.values_size());
        for(int i=0;i<lrf_feedback.values_size();i++)
        {
            // mm to metres
            lrf_msg.ranges.push_back((float)(lrf_feedback.values(i))/1000.0f);
        }

        lrf_publisher_.publish(lrf_msg);

        // Agent status and message
        std_msgs::UInt32 agent_status;
        agent_status.data = feedback->status_msg_data().status_val();
        agent_system_status_publisher_.publish(agent_status);
        std_msgs::String agent_msg;
        agent_msg.data = feedback->status_msg_data().message();
        agent_system_msg_publisher_.publish(agent_msg);
    }

    void runOdometry()
    {
        //ROS_INFO("Spinning odometry with delay %d ms",odom_delay_ms);
        while(node_alive_)
        {
            {
                std::lock_guard<std::mutex> guard(mtx);
                // Update odometry messages
                odom_node_.update_odom();
            }
            // Publish odometry messages
            odom_node_.publish_euler();
            odom_node_.publish_quat();
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_delay_ms));   
        }
    }


private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher odom_data_pub_euler;
    ros::Publisher odom_data_pub_quat;
    ros::Publisher lrf_publisher_;
    ros::Publisher agent_system_status_publisher_;
    ros::Publisher agent_system_msg_publisher_;
    std::string khepera_frame;
    std::thread odom_thread_;
    bool node_alive_;
    int odom_freq_hz;
    int odom_delay_ms;
    std::mutex mtx;
    el::Logger* logger; 
    std::string rid;
    OdomNode odom_node_;
};

#endif