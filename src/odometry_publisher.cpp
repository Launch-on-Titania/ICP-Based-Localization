#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_pub");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  tf::TransformListener listener;

  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce();             
      // check for incoming messages
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/odom_source",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      std::cout<<ex.what();
      ros::Duration(1.0).sleep();
    }

    
    

   

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = transform.stamp_;
    odom.header.frame_id = "map";

    //set the position
    
    odom.pose.pose.position.x = transform.getOrigin().x();
    odom.pose.pose.position.y = transform.getOrigin().y();
    odom.pose.pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), odom.pose.pose.orientation) ;

    //set the velocity
    odom.child_frame_id = "odom_source";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    //publish the message
    odom_pub.publish(odom);

    
    r.sleep();
  }
}