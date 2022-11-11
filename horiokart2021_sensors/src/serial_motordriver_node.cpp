#include "horiokart2021_sensors/serial_motordriver.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <fstream>
#include <time.h>
#include <vector>
#include <sstream>


using namespace std;
using namespace horiokart2021_sensors;


class SerialMotorDriverNode
{
    private:
        ros::NodeHandle n;
        ros::NodeHandle nh_private;

        SerialMotorDriver motordriver;

        string device_name;
        string twist_frame;
        string base_frame;

        bool r_dir, l_dir;
        float wheel_pitch;

        bool is_write_csv;
        string csv_name;
        ofstream ofs;

        ros::Subscriber serial_sub;

        const int TIME_TO_SLEEP = 10000; // usec
        const int MAX_SPEED = 4000; // mm/s

        ostringstream ss;

        SetSpeedRequest createSetSpeedRequest(const geometry_msgs::Twist vel);

    public:
        SerialMotorDriverNode();


        void init_ros_params();
        void twist_sub_cb(const geometry_msgs::Twist &vel);

        void write_log_csv();
        void run();
};

SerialMotorDriverNode::SerialMotorDriverNode()
:nh_private("~")
{

    init_ros_params();

    if(is_write_csv){
        if(csv_name == "")
        {
            char date[64];
            time_t t = ros::Time::now().sec;
            strftime(date, sizeof(date), "motor_%Y%m%d%a_%H%M%S.csv", localtime(&t));
            csv_name = date;

            ROS_INFO("Motor create CSV %s", csv_name.c_str());
        }
        ofs = ofstream(csv_name);
    }
    else{
        ROS_INFO("Motor No CSV");
    }

    motordriver = SerialMotorDriver(device_name, TIME_TO_SLEEP);
    if (!motordriver.isAlive())
    {
        ROS_ERROR("Serial Fail: cound not open %s", device_name.c_str());
    }
}

void SerialMotorDriverNode::init_ros_params()
{
    //Subscriber
    serial_sub = n.subscribe(twist_frame, 1, &SerialMotorDriverNode::twist_sub_cb, this); 


    nh_private.param<string>("device_name", device_name, "/dev/ttyACM0");
    nh_private.param<string>("twist_frame", twist_frame, "/cmd_vel");
    nh_private.param<string>("base_frame", base_frame, "base_footprint");

    nh_private.param<bool>("r_dir", r_dir, true);
    nh_private.param<bool>("l_dir", l_dir, true);

    nh_private.param<float>("wheel_pitch", 358);

    nh_private.param<bool>("csv", is_write_csv, true);
    nh_private.param<string>("csv_name", csv_name, "");

}


SetSpeedRequest SerialMotorDriverNode::createSetSpeedRequest(geometry_msgs::Twist vel)
{
    SetSpeedRequest req;

    float vel_x = vel.linear.x; // m/s
    float vel_y = vel.linear.y;
    float ang_z = vel.angular.z; // rad/s

    float d = wheel_pitch/2;// ピッチ358.0mm d127.3mm
    d = d/1000;//m

    //float ro = vel_x / ang_z;
    //float v_r = ((ro + d) * ang_z ) * 1000; // mm/s
    //float v_l = ((ro - d) * ang_z ) * 1000; // mm/s

    float v_r = (vel_x + d * ang_z ) * 1000; // mm/s
    float v_l = (vel_x - d * ang_z ) * 1000; // mm/s


    if(abs(v_r) > MAX_SPEED || abs(v_l) > MAX_SPEED){
        // todo ROSERROR
        printf("max speed");
        //return;
    }

    req.rightWheelSpeed = static_cast<int16_t>(v_r);
    req.leftWheelSpeed = static_cast<int16_t>(v_l);

    return req;
}

void SerialMotorDriverNode::twist_sub_cb(const geometry_msgs::Twist &vel)
{
    cout << "x:" << vel.linear.x;
    cout << ",th:" << vel.angular.z << endl;

    SetSpeedRequest req = createSetSpeedRequest(vel);

    MotorDriverResponse res = motordriver.setSpeedData(req);


    if(is_write_csv){

        vector<uint8_t> sendBuf = motordriver.encode(req);

        ofs << ros::Time::now().sec << "." << ros::Time::now().nsec << ",";

        ofs << float(vel.linear.x) << ",";
        ofs << float(vel.linear.y) << ",";
        ofs << float(vel.angular.z) << ",";

        ofs << int(req.rightWheelSpeed) << ",";
        ofs << int(req.leftWheelSpeed) << ",";

        ofs << ",";
        for(auto b:sendBuf){
            ofs << static_cast<int>(b) << ",";
        }
        ofs << ",";
        ofs << int(res.error) << ",";

        ofs << ",";
        for(auto b:res.raw){
            ofs << static_cast<int>(b) << ",";
        }
        ofs << endl;
    }

}

void SerialMotorDriverNode::write_log_csv()
{
    // if(is_write_csv){
    //     ofs << ros::Time::now().sec << "." << ros::Time::now().nsec << ",";
    //     ofs << float(vel_x) << ",";
    //     ofs << float(vel_y) << ",";
    //     ofs << float(ang_z) << ",";
    //     ofs << int(rspd) << ",";
    //     ofs << int(lspd) << ",";
    //     ofs << static_cast<int>(c_sum) << ",";
    //     ofs << ",";
    //     for(auto b:sendBuf){
    //         ofs << static_cast<int>(b) << ",";
    //     }
    //     ofs << endl;
    // }
}

void SerialMotorDriverNode::run()
{
    ROS_INFO("start serial motordriver");
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motordriver_node");

    SerialMotorDriverNode s_motor;
    s_motor.run();
}
