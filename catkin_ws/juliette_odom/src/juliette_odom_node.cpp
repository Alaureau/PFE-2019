#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <poll.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>

#define LEFT_ENCODER_PIN 23
#define RIGHT_ENCODER_PIN 24
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT 1000 // 1 second

// Structure for encoder
struct t_encoder {
	int fd;
	unsigned int count;
};

// Thread function
void *incCount(void *arg) {
	// Initialization
	struct t_encoder *encoderPt;
	encoderPt = (struct t_encoder *) arg;
	struct pollfd fdset[2];
	int nfds = 2;
	int timeout, rc;
	
	timeout = POLL_TIMEOUT;	

	// Polling loop
	while (1) {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = STDIN_FILENO;
		fdset[0].events = POLLIN;
			
		fdset[1].fd = encoderPt->fd;
		fdset[1].events = POLLPRI;

		rc = poll(fdset, nfds, timeout);      

		if (rc < 0) {
			perror("\npoll() failed!");
			exit(1);
		}
							
		// I NEED TO UNDERSTAND THIS, WE ONLY NEED TO DETECT AN INTERRUPT WHICH SHOULD BE EQUAL TO A STATE CHANGE. 
		if (fdset[1].revents & POLLPRI) {
			lseek(fdset[1].fd, 0, SEEK_SET);
			printf("\npoll() GPIO %d interrupt occurred\n", encoderPt->fd);
			// Increment the counter
			encoderPt->count++;
		}
	}
}

// Sets the pin to be an readable. Returns an int corresponding to a handle for the pin's value file
int setEncoderPin(unsigned int encoderPin){
	// Export the pin
	int fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("Unable to open gpio/export");
		exit(1);
	}
	
	if (write(fd, std::string.to_string(encoderPin), 2) != 2) {
		perror("Error writing to gpio/export");
		exit(1);
	}

	close(fd);
	
	// Set the the pin as input
	fd = open(SYSFS_GPIO_DIR "/gpio" + std::string.to_string(encoderPin) + "/direction", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open gpio/direction");
		exit(1);
	}

	if (write(fd, "in", 3) != 3) {
		perror("Error writing to gpio/direction");
		exit(1);
	}

	close(fd);
	
	// Open pin's value file
	fd = open(SYSFS_GPIO_DIR "/gpio" + std::string.to_string(encoderPin) + "/value", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open gpio/value");
		exit(1);
	}

	return fd;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	double pi = 3.141592;
	double r = 0.05; // Wheel radius
	double L = 0.2; // Distance between the two wheels
	double deltaEg = 0.0;
	double deltaEd = 0.0;
	double deltaV = 0.0;
	double deltaW = 0.0;
	
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	
	double v = 0.0;
	double w = 0.0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	// GPIO setting
	struct t_encoder leftEncoder;
	leftEncoder.fd = setEncoderPin(LEFT_ENCODER_PIN);
	leftEncoder.count = 0;
	
	struct t_encoder rightEncoder;
	rightEncoder.fd = setEncoderPin(RIGHT_ENCODER_PIN);
	rightEncoder.count = 0;

	//Phread create 
	int rc;
	pthread_t leftEncoderThread, rightEncoderThread; 
	
	rc = pthread_create(&leftEncoderThread, NULL, incCount, (void *)&leftEncoder);
	if (rc) {
		std::cout << "Error:unable to create thread," << rc << std::endl;
		exit(-1);
	}
	
	rc = pthread_create(&rightEncoderThread, NULL, incCount, (void *)&rightEncoder);
	if (rc) {
		std::cout << "Error:unable to create thread," << rc << std::endl;
		exit(-1);
	}

	ros::Rate r(1.0);
	while(n.ok()){
		ros::spinOnce(); // check for incoming messages
		current_time = ros::Time::now();

		/* Odometry */
		// Computing deltas
		deltaEg = (leftEncoder.count * pi) / 4;
		deltaEd = (rightEncoder.count * pi) / 4;
		deltaV = (r/2) * (deltaEg + deltaEd);
		deltaW = (r/(2*L)) * (deltaEd - deltaEg);

		// Reset deltas
		leftEncoder.count = 0;
		rightEncoder.count = 0;
		
		// Compute x, y and th
		x += deltaV * Math.cos(th);
		y += deltaV * Math.sin(th);
		th += th + deltaW;
		
		// Compute v and w
		double dt = (current_time - last_time).toSec();
		v = deltaV/dt;
		w = deltaW/dt;

		// Create quaternion from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		// Publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		// Send the transform
		odom_broadcaster.sendTransform(odom_trans);

		// Publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		// Set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// Set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = v;
		odom.twist.twist.angular.z = w;

		// Publish the message
		odom_pub.publish(odom);

		last_time = current_time;
		r.sleep();
	}

	// Cancel encoder threads and join
	pthread_cancel(leftEncoderThread);
	pthread_cancel(rightEncoderThread);
	pthread_join(leftEncoderThread, NULL);
	pthread_join(rightEncoderThread, NULL);
}
