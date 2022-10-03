#include "horiokart2021_sensors/serial_communicator.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <fstream>
#include <time.h>
#include <vector>


using namespace std;
using namespace horiokart2021_sensors;


class SerialOdom
{
    private:
        ros::NodeHandle n;

        //Publisher
        ros::Publisher serial_pub;

        ros::NodeHandle nh_private;

        string device_name;
        string odom_frame;
        string base_frame;
        int odom_pub_rate;

        bool is_write_csv;
        string csv_name;
        ofstream ofs;

        bool inv_x, inv_y, inv_th;

        int odom_ret_buf_size = 19;
        double odom_x = 0, odom_y = 0, odom_th = 0;
        double odom_vx = 0, odom_vy = 0, odom_vth = 0;
        double th = 0, vth = 0;

        SerialCommunicator serial;
        const vector<uint8_t> zeroBuf{0x24, 0x73};
        const vector<uint8_t> odomBuf{0x24, 0x75};
        uint8_t checksum = 0;

        const int TIME_TO_SLEEP = 2000; // usec

    public:
        SerialOdom();
        ~SerialOdom(){};


        void init_ros_params();
        int update_odom(vector<uint8_t>&);
        void publish_odom();

        void write_log_csv(int debug_status, vector<uint8_t> buf);
        void run();
};

SerialOdom::SerialOdom()
:nh_private("~")
{

    init_ros_params();

    if(is_write_csv){
        if(csv_name == "")
        {
            char date[64];
            time_t t = ros::Time::now().sec;
            strftime(date, sizeof(date), "odom_%Y%m%d%a_%H%M%S.csv", localtime(&t));
            csv_name = date;

            ROS_INFO("Odom create CSV %s", csv_name.c_str());
        }
        ofs = ofstream(csv_name);
    }
    else{
        ROS_INFO("Odom No CSV");
    }

    serial = SerialCommunicator(device_name);
    if(!serial.is_open_serial){
        ROS_ERROR("Serial Fail: cound not open %s", device_name.c_str());
    }


}

void SerialOdom::init_ros_params()
{
    ros::Publisher serial_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

    nh_private.param<string>("device_name", device_name, "/dev/ttyACM0");
    nh_private.param<string>("odom_frame", odom_frame, "odom");
    nh_private.param<string>("base_frame", base_frame, "base_footprint");
    nh_private.param<int>("odom_pub_rate", odom_pub_rate, 20);

    nh_private.param<bool>("inv_x", inv_x, false);
    nh_private.param<bool>("inv_y", inv_y, false);
    nh_private.param<bool>("inv_th", inv_th, false);

    nh_private.param<bool>("csv", is_write_csv, true);
    nh_private.param<string>("csv_name", csv_name, "");

}

int SerialOdom::update_odom(vector<uint8_t>& retbuf)
{
    checksum = 0;

    int rec=serial.serial_write(odomBuf);
    ROS_DEBUG("send:%#x %#x\n",odomBuf[0], odomBuf[1]);

    usleep(TIME_TO_SLEEP);
    if(rec<=0){
        ROS_ERROR("Odom write error");
        return 1;
    }

    // read
    retbuf = serial.serial_read();
    int recv_data = retbuf.size();
    ROS_DEBUG("recv %d \n", recv_data);

    if(recv_data<=0){
        ROS_ERROR("Odom receive size error");
        return 2; // 受信データサイズ0
    }

    ROS_DEBUG("recv:");
    for( const auto& b: retbuf)
    {
        ROS_DEBUG("%#x ", b);
    }
    ROS_DEBUG("\n");

    if(retbuf[0] != 0x24){
        ROS_ERROR("invalid command!\n");
        return 3; // 返り値変 0x24
    }
    if(retbuf[1] != 0x75){
        ROS_ERROR("invalid command\n");
        return 4; // 返り値変 0x75
    }

    checksum = serial.calc_checksum(vector<uint8_t>(retbuf.begin(), retbuf.end()-1));
    bool checksum_ok = (checksum == retbuf.back());

    if(!checksum_ok) {
        return 5; // checksum false
    }

    odom_x = double(((int32_t)retbuf[2])<<24 | ((int32_t)retbuf[3])<<16 | ((int32_t)retbuf[4])<<8 | ((int32_t)retbuf[5]))/1000;
    odom_y = double((int32_t)retbuf[6]<<24 | (int32_t)retbuf[7]<<16 | (int32_t)retbuf[8]<<8 | retbuf[9])/1000;
    odom_th = double((uint16_t)retbuf[10]<<8 | retbuf[11])/10000;

    odom_vx = double((int16_t)retbuf[12]<<8 | retbuf[13])/1000;
    odom_vy = double((int16_t)retbuf[14]<<8 | retbuf[15])/1000;
    odom_vth = double((int16_t)retbuf[16]<<8 | retbuf[17])/10000;

    if(inv_x) odom_x *= -1;
    if(inv_y) odom_y *= -1;
    if(inv_th) odom_th *= -1;

    return 0; // OK

}

void SerialOdom::publish_odom()
{
    double th = odom_th * 180 / 3.14;
    double vth = odom_vth * 180 / 3.14;
    ROS_DEBUG("x:%lf y:%lf th:%lf \n", odom_x, odom_y, th);
    ROS_DEBUG("vx:%lf vy:%lf vth%lf \n", odom_vx, odom_vy, vth);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);

    ros::Time current_time = ros::Time::now();
    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;

    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = odom_vx;
    odom.twist.twist.linear.y = odom_vy;
    odom.twist.twist.angular.z = odom_vth;

    serial_pub.publish(odom);
}

void SerialOdom::write_log_csv(int debug_status, vector<uint8_t> retbuf)
{
    if(is_write_csv){
        char date[64];
        time_t t = ros::Time::now().nsec / 1000000; // sec
        strftime(date, sizeof(date), "%H%M%S", localtime(&t));
        char output_date[32];
        snprintf(output_date, 32, "%s.%03d", date, static_cast<int>(t));

        ofs << output_date << ",";
        ofs << odom_x << ",";
        ofs << odom_y << ",";
        ofs << odom_th << ",";
        ofs << odom_x << ",";
        ofs << odom_y << ",";
        ofs << odom_th << ",";
        ofs << ",";
        ofs << static_cast<int>(checksum) << ",";
        ofs << debug_status << ",";
        ofs << ",";
        for(auto& b:retbuf){
            ofs << static_cast<int>(b) << ",";
        }
        ofs << endl;
    }

}

void SerialOdom::run()
{
    ROS_INFO("start serial odom");

    vector<uint8_t> ret = serial.serial_readwrite(zeroBuf);
    if(ret.empty()){
        ROS_ERROR("Odom error zero reset!");
    }
    else{
        ROS_INFO("Odom zero reset");
    }

    ros::Rate loop_rate(odom_pub_rate); 
    while (ros::ok()){

        vector<uint8_t> retbuf;
        int debug_status = this->update_odom(retbuf);
            
        // とりあえずpublish
        publish_odom();
        // -----------

        ros::spinOnce();
        loop_rate.sleep();
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_odom");

    SerialOdom s_odom;
    s_odom.run();
}
