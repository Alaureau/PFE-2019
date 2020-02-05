#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main(int argc, char** argv){
  ros::init(argc, argv, "juliette_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  double laser_height = 0.105;
  double base_height = 0.127;

  while(n.ok()){
	  
	// base_link -> laser
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "laser"));
        
    // base_link -> ultrasonic_0
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_0"));
    // base_link -> ultrasonic_1
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_1"));
    // base_link -> ultrasonic_2
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_2"));
    // base_link -> ultrasonic_3
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_3"));
    // base_link -> ultrasonic_4
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_4"));
    // base_link -> ultrasonic_5
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_5"));
	// base_link -> ultrasonic_6
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_6"));
	// base_link -> ultrasonic_7
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.168)),
        ros::Time::now(),"base_link", "ultrasonic_7"));
    r.sleep();
  }
}
