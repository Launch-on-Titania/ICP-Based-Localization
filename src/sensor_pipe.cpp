#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include "utm.hpp"

class SensorPipe
{
public:
    ros::NodeHandle nh;
    std::string imu_topic = "/simulated_imu";
    std::string rtk_odom_topic = "/rtk_odom";
    
    ros::Publisher imu_publisher; 
    sensor_msgs::Imu imu_buffer;
    geometry_msgs::Vector3 position_buffer;
    geometry_msgs::Vector3 position_covariance;
    double yaw_buffer;
    double forward_vel_buffer;

    ros::Publisher rtk_odom_publisher;
    nav_msgs::Odometry rtk_odom_buffer;

    std::string fix_topic = "/fix"; // GPS sensors_msgs/NavFixMsg
    ros::Subscriber fix_subscriber;

    std::string vel_topic = "/vel"; // geometry_msgs/TwistStamped
    ros::Subscriber vel_subscriber;

public:
    void init_params()
    {
        this->imu_buffer.header.frame_id = "inertial_sensor";
    }
    void init_publishers()
    {
        this->imu_publisher = this->nh.advertise<sensor_msgs::Imu>(this->imu_topic, 1);
        this->rtk_odom_publisher = this->nh.advertise<nav_msgs::Odometry>(this->rtk_odom_topic, 1);
    }
    void init_subscribers()
    {
        this->fix_subscriber = this->nh.subscribe(
            this->fix_topic,
            10,
            &SensorPipe::fix_cb,
            this
        );
        this->vel_subscriber = this->nh.subscribe(
            this->vel_topic,
            10,
            &SensorPipe::vel_cb,
            this
        );
    }

    void fix_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        double x, y;
        geometry_msgs::Vector3 position_buffer;
        char UTM_zone = UTM::UTMLetterDesignator(msg->latitude);
        UTM::LLtoUTM(
            msg->latitude, msg->longitude,
            x, y, // +x as direct north
            &UTM_zone
        );
        position_buffer.x = x; 
        position_buffer.y = y; 
        position_buffer.z = msg->altitude;
        

        this->position_buffer = position_buffer;
        this->position_covariance.x = msg->position_covariance[0];
        this->position_covariance.y = msg->position_covariance[1];
        this->position_covariance.z = msg->position_covariance[2];
    }
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        float time_interval = msg->header.stamp.sec - this->imu_buffer.header.stamp.sec 
            + (msg->header.stamp.nsec - this->imu_buffer.header.stamp.nsec) / 1000;
        double yaw = std::atan2(msg->twist.linear.y, msg->twist.linear.x);
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        double forward_vel = std::sqrt(
            msg->twist.linear.x * msg->twist.linear.x +
            msg->twist.linear.y * msg->twist.linear.y
        );

        
        this->imu_buffer.angular_velocity = msg->twist.angular;
        this->imu_buffer.angular_velocity.z = (yaw - this->yaw_buffer) / time_interval;
        this->imu_buffer.angular_velocity_covariance[0] = 100.;
        this->imu_buffer.angular_velocity_covariance[4] = 100.;
        this->imu_buffer.angular_velocity_covariance[8] = 100.;
        
        this->imu_buffer.linear_acceleration.x = (forward_vel - this->forward_vel_buffer) / time_interval;
        this->imu_buffer.linear_acceleration.y = forward_vel * this->imu_buffer.angular_velocity.z;
        this->imu_buffer.linear_acceleration.z = 0.;
        this->imu_buffer.linear_acceleration_covariance[0] = 100.;
        this->imu_buffer.linear_acceleration_covariance[4] = 100.;
        this->imu_buffer.linear_acceleration_covariance[8] = 100.;

        this->imu_buffer.orientation.x = q.x();
        this->imu_buffer.orientation.y = q.y();
        this->imu_buffer.orientation.z = q.z();
        this->imu_buffer.orientation.w = q.w();
        this->imu_buffer.orientation_covariance[0] = 100.;
        this->imu_buffer.orientation_covariance[4] = 100.;
        this->imu_buffer.orientation_covariance[8] = 10.;
        
        this->imu_buffer.header.stamp = msg->header.stamp;
        this->imu_publisher.publish(this->imu_buffer);

        this->rtk_odom_buffer.header.stamp = msg->header.stamp;
        this->rtk_odom_buffer.header.frame_id = "odom";
        this->rtk_odom_buffer.child_frame_id = "inertial_sensor";
        this->rtk_odom_buffer.pose.pose.position.x = this->position_buffer.x;
        this->rtk_odom_buffer.pose.pose.position.y = this->position_buffer.y;
        this->rtk_odom_buffer.pose.pose.position.z = this->position_buffer.z;
        this->rtk_odom_buffer.pose.pose.orientation = this->imu_buffer.orientation;
        this->rtk_odom_buffer.pose.covariance[0] = this->position_covariance.x;
        this->rtk_odom_buffer.pose.covariance[7] = this->position_covariance.y;
        this->rtk_odom_buffer.pose.covariance[14] = this->position_covariance.z;
        this->rtk_odom_buffer.pose.covariance[21] = 100.;
        this->rtk_odom_buffer.pose.covariance[28] = 100.;
        this->rtk_odom_buffer.pose.covariance[35] = 10.;
        this->rtk_odom_buffer.twist.twist.linear.x = forward_vel;
        this->rtk_odom_buffer.twist.twist.linear.y = 0.;
        this->rtk_odom_buffer.twist.twist.linear.z = 0.;
        this->rtk_odom_buffer.twist.twist.angular.x = 0.;
        this->rtk_odom_buffer.twist.twist.angular.y = 0.;
        this->rtk_odom_buffer.twist.twist.angular.z = this->imu_buffer.angular_velocity.z;
        this->rtk_odom_buffer.twist.covariance[0] = std::sqrt(this->position_covariance.x + this->position_covariance.y);
        this->rtk_odom_buffer.twist.covariance[7] = 1.;
        this->rtk_odom_buffer.twist.covariance[14] = std::sqrt(this->position_covariance.z);
        this->rtk_odom_buffer.twist.covariance[21] = 100.;
        this->rtk_odom_buffer.twist.covariance[28] = 100.;
        this->rtk_odom_buffer.twist.covariance[35] = 100.;
        this->rtk_odom_publisher.publish(this->rtk_odom_buffer);

        this->forward_vel_buffer = forward_vel;
        this->yaw_buffer = yaw;
    }

    SensorPipe(ros::NodeHandle &nh)
    : nh(nh)
    {
        this->init_params();
        this->init_publishers();
        this->init_subscribers();
    }
};

int main(int argc, char **argv)
{
    std::string robot_namespace (argv[1]);
    ros::init(argc, argv, robot_namespace);
    ros::NodeHandle nh ("~");

    auto pipe = SensorPipe(nh);
    ros::spin();
    return 0;
}