
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <iomanip>

#include <fstream>
#include <time.h>
#include <unistd.h>


using namespace std;


ros::Time current_time;

string device_name;
string odom_frame;
string base_frame;
int odom_pub_rate;
int vel_pub_rate;
bool is_write_csv;
string csv_name;
ofstream ofs;

bool inv_x;
bool inv_y;
bool inv_th;

void odom2tf_cb(const nav_msgs::Odometry msg)
{
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(msg.pose.pose.orientation);
    static tf::TransformBroadcaster odom_broadcaster;

    current_time = ros::Time::now();
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame;
    odom_trans.child_frame_id = base_frame;

    odom_trans.transform.translation.x = msg.pose.pose.position.x;
    odom_trans.transform.translation.y = msg.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = msg.pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2tf");
    ros::NodeHandle n;

    ros::NodeHandle nh_private("~");

    current_time = ros::Time::now();


    nh_private.param<string>("odom_frame", odom_frame, "odom");
    nh_private.param<string>("base_frame", base_frame, "base_footprint");
    nh_private.param<int>("odom_pub_rate", odom_pub_rate, 20);
    nh_private.param<int>("vel_pub_rate", vel_pub_rate, 20);

    ros::Subscriber sub = n.subscribe("odom", 1, odom2tf_cb); //<nav_msgs::Odometry>("odom", 1000);


    ros::spin();
    return 0;
}
