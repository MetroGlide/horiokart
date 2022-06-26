#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


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
void serial_callback(const std_msgs::String& serial_msg){

    // int rec=write(fd1,serial_msg.data.c_str(),serial_msg.data.size());
    // if(rec>=0)printf("send:%s\n",serial_msg.data.c_str());
    // else{
    //     ROS_ERROR_ONCE("Serial Fail: cound not write");
    // }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motordriver");
    ros::NodeHandle n;

    //Publisher
    ros::Publisher serial_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    //Subscriber
    ros::Subscriber serial_sub = n.subscribe("Serial_out", 10, serial_callback); 

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    char device_name[]="/dev/ttyACM0";
    fd1=open_serial(device_name);

    if(fd1<0){
        ROS_ERROR("Serial Fail: cound not open %s", device_name);
        printf("Serial Fail\n");
        ros::shutdown();
    }


    ros::Rate loop_rate(20); 
    while (ros::ok()){

        char odomBuf[] = {0x24, 0x75};

        int rec=write(fd1,odomBuf, sizeof(odomBuf));
        printf("send:%#x %#x\n",odomBuf[0], odomBuf[1]);
        if(rec>=0){
            uint8_t retbuf[64]={0};
            int recv_data=read(fd1, retbuf, sizeof(retbuf));
            printf("recv %d \n", recv_data);

            if(recv_data>0){
                printf("recv:");
                for(int i=0;i<recv_data;i++){
                    printf("%#x ", retbuf[i]);
                }
                printf("\n");

                if(retbuf[0] == 0x24){
                    std_msgs::String serial_msg; 
        
                    if(retbuf[1] == 0x75){
                        printf("recv odom \n");
                        double odom_x = double(((int32_t)retbuf[2])<<24 | ((int32_t)retbuf[3])<<16 | ((int32_t)retbuf[4])<<8 | ((int32_t)retbuf[5]))/1000;
                        double odom_y = double((int32_t)retbuf[6]<<24 | (int32_t)retbuf[7]<<16 | (int32_t)retbuf[8]<<8 | retbuf[9])/1000;
                        double odom_th = double((uint16_t)retbuf[10]<<8 | retbuf[11])/10000;

                        double odom_vx = double((int16_t)retbuf[12]<<8 | retbuf[13])/1000;
                        double odom_vy = double((int16_t)retbuf[14]<<8 | retbuf[15])/1000;
                        double odom_vth = double((int16_t)retbuf[16]<<8 | retbuf[17])/10000;

                        double th = odom_th * 180 / 3.14;
                        double vth = odom_vth * 180 / 3.14;
                        printf("x:%lf y:%lf th:%lf \n", odom_x, odom_y, th);
                        printf("vx:%lf vy:%lf vth%lf \n", odom_vx, odom_vy, vth);

                        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);

                        current_time = ros::Time::now();
                        
                        geometry_msgs::TransformStamped odom_trans;
                        odom_trans.header.stamp = current_time;
                        odom_trans.header.frame_id = "odom";
                        odom_trans.child_frame_id = "base_footprint";

                        odom_trans.transform.translation.x = odom_x;
                        odom_trans.transform.translation.y = odom_y;
                        odom_trans.transform.translation.z = 0.0;
                        odom_trans.transform.rotation = odom_quat;

                        odom_broadcaster.sendTransform(odom_trans);

                        nav_msgs::Odometry odom;
                        odom.header.stamp = current_time;
                        odom.header.frame_id = "odom";

                        odom.pose.pose.position.x = odom_x;
                        odom.pose.pose.position.y = odom_y;
                        odom.pose.pose.position.z = 0.0;
                        odom.pose.pose.orientation = odom_quat;

                        odom.child_frame_id = "base_footprint";
                        odom.twist.twist.linear.x = odom_vx;
                        odom.twist.twist.linear.y = odom_vy;
                        odom.twist.twist.angular.z = odom_vth;

                        serial_pub.publish(odom);

                    }
                    else
                    {
                        printf("invalid command\n");

                    }
                }
                else{
                    printf("invalid command!\n");
                }
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
