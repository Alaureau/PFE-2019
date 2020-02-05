#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main(int argc, char** argv){
  ros::init(argc, argv, "juliette_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  double laser_height = 0.084;
  double base_height = 0.127;
  double pi = 3.141592;
  
  double base_to_laser_height = (base_height / 2) + laser_height;
  
  // Yaw rotations
  double right_yaw = -pi/2;
  double rear_yaw = pi;
  double left_yaw = pi/2;
  
  // Quaternion rotations
  double right_w = cos(right_yaw * 0.5);
  double right_z = sin(right_yaw * 0.5);
  
  double rear_w = cos(rear_yaw * 0.5);
  double rear_z = sin(rear_yaw * 0.5);
  
  double left_w = cos(left_yaw * 0.5);
  double left_z = sin(left_yaw * 0.5);
  
  double front_and_back_ultrasonic_length = 0.04;
  double side_ultrasonic_length = 0.135;
  
  double base_to_front_and_back_ultrasonic_width = 0.045;
  double base_to_side_ultrasonic_width = 0.075;
  
  double base_width = 0.12;
  double base_length = 0.32;
  
  double ultrasonic_height = 0.025;
  
  // Translations
  // Front
  double base_to_ultrasonic_0_x = (base_length / 2) + front_and_back_ultrasonic_length;
  double base_to_ultrasonic_0_y = - base_to_front_and_back_ultrasonic_width;
  
  double base_to_ultrasonic_1_x = (base_length / 2) + front_and_back_ultrasonic_length;
  double base_to_ultrasonic_1_y = + base_to_front_and_back_ultrasonic_width;
  
  // Right
  double base_to_ultrasonic_2_x = (base_width / 2) + side_ultrasonic_length;
  double base_to_ultrasonic_2_y = - base_to_side_ultrasonic_width;
  
  double base_to_ultrasonic_3_x = (base_width / 2) + side_ultrasonic_length;
  double base_to_ultrasonic_3_y = + base_to_side_ultrasonic_width;
  
  // Rear
  double base_to_ultrasonic_4_x = (base_length / 2) + front_and_back_ultrasonic_length;
  double base_to_ultrasonic_4_y = - base_to_side_ultrasonic_width;
  
  double base_to_ultrasonic_5_x = (base_length / 2) + front_and_back_ultrasonic_length;
  double base_to_ultrasonic_5_y = + base_to_side_ultrasonic_width;
  
  // Left
  double base_to_ultrasonic_6_x = (base_width / 2) + side_ultrasonic_length;
  double base_to_ultrasonic_6_y = - base_to_side_ultrasonic_width;
  
  double base_to_ultrasonic_7_x = (base_width / 2) + side_ultrasonic_length;
  double base_to_ultrasonic_7_y = + base_to_side_ultrasonic_width;
  

  while(n.ok()){
	  
	// base_link -> laser
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, base_to_laser_height)),
        ros::Time::now(),"base_link", "laser"));
        
    // base_link -> ultrasonic_0
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(base_to_ultrasonic_0_x, base_to_ultrasonic_0_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_0"));
    // base_link -> ultrasonic_1
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(base_to_ultrasonic_1_x, base_to_ultrasonic_1_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_1"));
    // base_link -> ultrasonic_2
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(right_w, 0, 0, right_z), tf::Vector3(base_to_ultrasonic_2_x, base_to_ultrasonic_2_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_2"));
    // base_link -> ultrasonic_3
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(right_w, 0, 0, right_z), tf::Vector3(base_to_ultrasonic_3_x, base_to_ultrasonic_3_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_3"));
    // base_link -> ultrasonic_4
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(rear_w, 0, 0, rear_z), tf::Vector3(base_to_ultrasonic_4_x, base_to_ultrasonic_4_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_4"));
    // base_link -> ultrasonic_5
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(rear_w, 0, 0, rear_z), tf::Vector3(base_to_ultrasonic_5_x, base_to_ultrasonic_5_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_5"));
	// base_link -> ultrasonic_6
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(left_w, 0, 0, left_z), tf::Vector3(base_to_ultrasonic_6_x, base_to_ultrasonic_6_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_6"));
	// base_link -> ultrasonic_7
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(left_w, 0, 0, left_z), tf::Vector3(base_to_ultrasonic_7_x, base_to_ultrasonic_7_y, ultrasonic_height)),
        ros::Time::now(),"base_link", "ultrasonic_7"));
    r.sleep();
  }
}
