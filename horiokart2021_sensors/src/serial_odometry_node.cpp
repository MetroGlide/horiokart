#include "horiokart2021_sensors/serial_odometry.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <fstream>
#include <time.h>
#include <vector>
#include <sstream>
#include <math.h>


using namespace std;
using namespace horiokart2021_sensors;


class SerialOdometryNode
{
    private:
        ros::NodeHandle n;
        ros::NodeHandle nh_private;

        //Publisher
        ros::Publisher serial_pub;

        SerialOdometry odometry;

        string device_name;
        string odom_frame;
        string base_frame;
        int odom_pub_rate;

        bool is_write_csv;
        string csv_name;
        ofstream ofs;

        bool inv_x, inv_y, inv_th;
        bool alwaysPublish;

        const int TIME_TO_SLEEP = 100000; // usec

        ostringstream ss;

        OdometryData currentData;
        OdometryData recentNomalData;

    public:
        SerialOdometryNode();


        void init_ros_params();
        void update_odom();
        void publish_odom();

        void write_log_csv();
        void run();
};

SerialOdometryNode::SerialOdometryNode()
:nh_private("~")
{

    init_ros_params();

    if(is_write_csv){
        if(csv_name == "")
        {
            char date[64];
            time_t t = ros::WallTime::now().sec;
            strftime(date, sizeof(date), "odom_%Y%m%d%a_%H%M%S.csv", localtime(&t));
            csv_name = string(date);
            cout << date << endl;

            ROS_INFO("Odom create CSV %s", csv_name.c_str());
        }
        ofs = ofstream(csv_name);
    }
    else{
        ROS_INFO("Odom No CSV");
    }

    odometry = SerialOdometry(device_name, TIME_TO_SLEEP);
    if (!odometry.isAlive())
    {
        ROS_ERROR("Serial Fail: cound not open %s", device_name.c_str());
    }
}

void SerialOdometryNode::init_ros_params()
{
    serial_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

    nh_private.param<string>("device_name", device_name, "/dev/ttyACM0");
    nh_private.param<string>("odom_frame", odom_frame, "odom");
    nh_private.param<string>("base_frame", base_frame, "base_footprint");
    nh_private.param<int>("odom_pub_rate", odom_pub_rate, 20);

    nh_private.param<bool>("inv_x", inv_x, false);
    nh_private.param<bool>("inv_y", inv_y, false);
    nh_private.param<bool>("inv_th", inv_th, false);

    nh_private.param<bool>("always_publish", alwaysPublish, true);

    nh_private.param<bool>("csv", is_write_csv, true);
    nh_private.param<string>("csv_name", csv_name, "");

}

void SerialOdometryNode::update_odom()
{
    currentData = odometry.getData();

    ss.str("");
    ss << "recv: ";
    for( const auto& b: currentData.raw)
    {
        ss << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b) << " ";

    }
    ROS_DEBUG("%s", ss.str().c_str());
    if (currentData.error != SerialError::NoError)
    {
        // todo switch
        ROS_ERROR("ERROR code %d", static_cast<int>(currentData.error));
    }
    else{
        if(inv_x) currentData.x *= -1;
        if(inv_y) currentData.y *= -1;
        if(inv_th) currentData.th *= -1;

        recentNomalData = currentData;
    }
}

void SerialOdometryNode::publish_odom()
{
    if(currentData.error != SerialError::NoError && !alwaysPublish)
        return;

    // double th = recentNomalData.th * 180 / 3.14;
    // double vth = recentNomalData.vth * 180 / 3.14;
    double th = recentNomalData.th ;
    double vth = recentNomalData.vth ;
    ROS_DEBUG("x:%lf y:%lf th:%lf ", recentNomalData.x, recentNomalData.y, th);
    ROS_DEBUG("vx:%lf vy:%lf vth%lf ", recentNomalData.vx, recentNomalData.vy, vth);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(recentNomalData.th);

    ros::Time current_time = ros::Time::now();
    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;

    odom.pose.pose.position.x = recentNomalData.x;
    odom.pose.pose.position.y = recentNomalData.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    double vx = sqrt(pow(recentNomalData.vx, 2) + pow(recentNomalData.vy, 2));
    odom.child_frame_id = base_frame;
    // odom.twist.twist.linear.x = recentNomalData.vx;
    // odom.twist.twist.linear.y = recentNomalData.vy;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;

    serial_pub.publish(odom);
}

void SerialOdometryNode::write_log_csv()
{
    if(is_write_csv){

        ofs << ros::WallTime::now().sec << "." << ros::WallTime::now().nsec << ",";
        ofs << currentData.x << ",";
        ofs << currentData.y << ",";
        ofs << currentData.th << ",";
        ofs << currentData.vx << ",";
        ofs << currentData.vy << ",";
        ofs << currentData.vth << ",";
        ofs << ",";
        ofs << static_cast<int>(currentData.error) << ",";
        ofs << ",";
        for(auto& b:currentData.raw){
            ofs << static_cast<int>(b) << ",";
        }
        ofs << endl;
    }

}

void SerialOdometryNode::run()
{
    ROS_INFO("start serial odom");

    SerialError e = odometry.sendZeroReset(3);
    if (e == SerialError::NoError)
    {
        ROS_INFO("Odom zero reset");
    }
    else{
        ROS_ERROR("Odom error zero reset!");
    }

    ros::Rate loop_rate(odom_pub_rate); 
    while (ros::ok()){

        update_odom();
        publish_odom();
        write_log_csv();

        ros::spinOnce();
        loop_rate.sleep();
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_odometry");

    SerialOdometryNode s_odom;
    s_odom.run();
}
