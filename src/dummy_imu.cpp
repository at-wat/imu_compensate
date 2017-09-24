#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


class dummy_imu
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_odom;
	ros::Publisher pub_imu;
		
	nav_msgs::Odometry odom;

	void cb_odom(const nav_msgs::Odometry::Ptr &msg)
	{
		odom = *msg;
	}
public:
	dummy_imu():
		nh("~")
	{
		sub_odom = nh.subscribe("odom_raw", 1, &dummy_imu::cb_odom, this);
		pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 2);
	}
	void spin()
	{
		ros::Rate r(50);
		while(ros::ok())
		{
			r.sleep();
			ros::spinOnce();
			sensor_msgs::Imu imu;
			imu.header.frame_id = "imu_link";
			imu.header.stamp = ros::Time::now();
			imu.orientation = odom.pose.pose.orientation;
			imu.linear_acceleration.z = 9.8;
			imu.angular_velocity.z = odom.twist.twist.angular.z;
			pub_imu.publish(imu);
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dummy_imu");
	
	dummy_imu imu;

	imu.spin();

	return 0;
}


