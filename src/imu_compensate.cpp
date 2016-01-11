#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>


class imu_compensate
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_imu;
	ros::Publisher pub_imu;
		
	nav_msgs::Odometry odom;

	double gyro_zero[3];
	int cnt;

	void cb_imu(const sensor_msgs::Imu::Ptr &msg)
	{
		auto imu = *msg;
		double acc = sqrt(pow(imu.linear_acceleration.x, 2.0)
				+ pow(imu.linear_acceleration.y, 2.0)
				+ pow(imu.linear_acceleration.z, 2.0));

		if(fabs(odom.twist.twist.angular.z) < 0.01 && 
				fabs(odom.twist.twist.linear.x) < 0.01 &&
				fabs(acc - 9.8) < 0.5 &&
				fabs(imu.angular_velocity.x) < 0.05 &&
				fabs(imu.angular_velocity.y) < 0.05 &&
				fabs(imu.angular_velocity.z) < 0.05)
		{
			cnt ++;
			if(cnt > 20)
			{
				double k = 0.01;
				gyro_zero[0] = gyro_zero[0] * (1.0 - k) + imu.angular_velocity.x * k;
				gyro_zero[1] = gyro_zero[1] * (1.0 - k) + imu.angular_velocity.y * k;
				gyro_zero[2] = gyro_zero[2] * (1.0 - k) + imu.angular_velocity.z * k;
			}
		}
		else
		{
			cnt = 0;
		}
		imu.angular_velocity.x -= gyro_zero[0];
		imu.angular_velocity.y -= gyro_zero[1];
		imu.angular_velocity.z -= gyro_zero[2];
		pub_imu.publish(imu);
	}
	void cb_odom(const nav_msgs::Odometry::Ptr &msg)
	{
		odom = *msg;
	}
public:
	imu_compensate():
		nh("~")
	{
		sub_odom = nh.subscribe("odom_raw", 1, &imu_compensate::cb_odom, this);
		sub_imu = nh.subscribe("imu_raw", 1, &imu_compensate::cb_imu, this);
		pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 2);
		cnt = 0;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "imu_compensate");
	
	imu_compensate imu;

	ros::spin();

	return 0;
}


