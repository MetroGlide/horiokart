#include <ros/ros.h>
#include <std_msgs/String.h>
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

int open_serial(const char *device_name){
    // int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    int fd1=open(device_name, O_RDWR | O_NOCTTY );
    if(fd1 < 0){
        printf("serial open failed...\n");
    }
    fcntl(fd1, F_SETFL,0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd1,&conf_tio);
    //set baudrate
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    //non blocking
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    //store configuration
    tcsetattr(fd1,TCSANOW,&conf_tio);
    return fd1;
}

int fd1;

uint8_t calc_checksum(uint8_t *buf, int buf_size){
    uint8_t sum = 0;
    
    for(int i = 0; i < buf_size-1; i++)
    {
        sum += buf[i];
    }
    return sum;
}
int serial_write(int fd, uint8_t buf[], int buf_size)
{
    if(fd >= 0){
        return write(fd, buf, buf_size);
    }
    else{
        return 0;
    }
}
int serial_read(int fd, uint8_t buf[], int buf_size)
{
    if(fd >= 0){
        return read(fd, buf, buf_size);
    }
    else{
        return 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_odom");
    ros::NodeHandle n;

    //Publisher
    ros::Publisher serial_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    ros::NodeHandle nh_private("~");

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

    nh_private.param<string>("device_name", device_name, "/dev/ttyACM0");
    nh_private.param<string>("odom_frame", odom_frame, "odom");
    nh_private.param<string>("base_frame", base_frame, "base_footprint");
    nh_private.param<int>("odom_pub_rate", odom_pub_rate, 20);
    nh_private.param<int>("vel_pub_rate", vel_pub_rate, 20);

    nh_private.param<bool>("inv_x", inv_x, false);
    nh_private.param<bool>("inv_y", inv_y, false);
    nh_private.param<bool>("inv_th", inv_th, false);

    nh_private.param<bool>("csv", is_write_csv, true);
    nh_private.param<string>("csv_name", csv_name, "");
    if(is_write_csv){
        if(csv_name == "")
        {
            char date[64];
            time_t t = ros::Time::now().sec;
            strftime(date, sizeof(date), "odom_%Y%m%d%a_%H%M%S.csv", localtime(&t));
            csv_name = date;
            cout << "Odom Create CSV" << endl;
            cout << date << endl;
            cout << endl;
        }
        ofs = ofstream(csv_name);
    }
    else{
        cout << "Odom No debug" << endl << endl;
    }

    uint8_t odomBuf[] = {0x24, 0x75};
    int odom_ret_buf_size = 19;
    double odom_x = 0, odom_y = 0, odom_th = 0;
    double odom_vx = 0, odom_vy = 0, odom_vth = 0;
    double th = 0, vth = 0;
    uint8_t checksum = 0;
    bool checksum_ok;
    int debug_status;


    fd1=open_serial(device_name.c_str());

    if(fd1<0){
        ROS_ERROR("Serial Fail: cound not open %s", device_name.c_str());
        printf("Serial Fail\n");
        // ros::shutdown();
    }

    uint8_t zeroBuf[] = {0x24, 0x73};
    // int rec=write(fd1, zeroBuf, sizeof(zeroBuf));
    int rec=serial_write(fd1, zeroBuf, sizeof(zeroBuf));
    // printf("send:%#x %#x\n",zeroBuf[0], zeroBuf[1]);
    if(rec>=0){
        uint8_t retbuf[64]={0};
        int recv_data=serial_read(fd1, retbuf, sizeof(retbuf));
        printf("recv %d \n", recv_data);
    }

    const int TIME_TO_SLEEP = 1000; // usec

    ros::Rate loop_rate(odom_pub_rate); 
    while (ros::ok()){

        // TODO: バッファクリア
        int rec=serial_write(fd1, odomBuf, sizeof(odomBuf));
        // printf("send:%#x %#x\n",odomBuf[0], odomBuf[1]);

        usleep(TIME_TO_SLEEP);
        if(rec>=0){
            uint8_t retbuf[64]={0};
            int recv_data=serial_read(fd1, retbuf, sizeof(retbuf));
            // printf("recv %d \n", recv_data);

            if(recv_data>0){
                // printf("recv:");
                // for(int i=0;i<recv_data;i++){
                //     printf("%#x ", retbuf[i]);
                // }
                // printf("\n");

                if(retbuf[0] == 0x24){
        
                    if(retbuf[1] == 0x75){
                        checksum = calc_checksum(retbuf, odom_ret_buf_size);
                        checksum_ok = (checksum == retbuf[odom_ret_buf_size-1]);

                        if(checksum_ok) {
                            debug_status = 0; // OK

                            // printf("recv odom \n");
                            odom_x = double(((int32_t)retbuf[2])<<24 | ((int32_t)retbuf[3])<<16 | ((int32_t)retbuf[4])<<8 | ((int32_t)retbuf[5]))/1000;
                            odom_y = double((int32_t)retbuf[6]<<24 | (int32_t)retbuf[7]<<16 | (int32_t)retbuf[8]<<8 | retbuf[9])/1000;
                            odom_th = double((uint16_t)retbuf[10]<<8 | retbuf[11])/10000;

                            odom_vx = double((int16_t)retbuf[12]<<8 | retbuf[13])/1000;
                            odom_vy = double((int16_t)retbuf[14]<<8 | retbuf[15])/1000;
                            odom_vth = double((int16_t)retbuf[16]<<8 | retbuf[17])/10000;

                            if(inv_x) odom_x *= -1;
                            if(inv_y) odom_y *= -1;
                            if(inv_th) odom_th *= -1;
                        }else{
                            debug_status = 1; // checksum false
                        }


                    }
                    else
                    {
                        printf("invalid command\n");
                        printf("publish before data\n");

                        debug_status = 3; // 返り値変 0x75
                    }

                }
                else{
                    printf("invalid command!\n");

                    debug_status = 4; // 返り値変 0x24
                }
            }
            else{
                debug_status = 2; // 受信データサイズ0
            }
            
            // 何かしら返ってきてたらとりあえずpublish
            double th = odom_th * 180 / 3.14;
            double vth = odom_vth * 180 / 3.14;
            printf("x:%lf y:%lf th:%lf \n", odom_x, odom_y, th);
            printf("vx:%lf vy:%lf vth%lf \n", odom_vx, odom_vy, vth);

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);

            current_time = ros::Time::now();
            
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = odom_frame;
            odom_trans.child_frame_id = base_frame;

            odom_trans.transform.translation.x = odom_x;
            odom_trans.transform.translation.y = odom_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);

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
            // -----------

            if(is_write_csv){
                char date[64];
                time_t t = ros::Time::now().nsec;
                strftime(date, sizeof(date), "%H%M%S", localtime(&t));
                char output_date[32];
                snprintf(output_date, 32, "%s.%03d", date, static_cast<int>(t)/1000000);

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
                for(auto b:retbuf){
                    ofs << static_cast<int>(b) << ",";
                }
                ofs << endl;
            }
        }
        else{
            ROS_ERROR_ONCE("Serial Fail: cound not write");
        }

        ros::spinOnce();
        loop_rate.sleep();
    } 
    return 0;
}
