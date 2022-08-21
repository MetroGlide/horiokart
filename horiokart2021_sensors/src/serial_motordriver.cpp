#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>

#include <fstream>
#include <time.h>


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

uint8_t calc_checksum(uint8_t *buf, int buf_size){
    uint8_t sum = 0;
    
    for(int i = 0; i < buf_size-1; i++)
    {
        sum += buf[i];
    }
    return sum;
}

// 
int serial_write(int fd, uint8_t buf[], int buf_size){
    if(fd >= 0){
        return write(fd, buf, buf_size);
    }
    else{
        return 0;
    }
}
int serial_read(int fd, uint8_t buf[], int buf_size){
    if(fd >= 0){
        return read(fd, buf, buf_size);
    }
    else{
        return 0;
    }
}

void buf_print(uint8_t buf[], int buf_size){
    for(int i=0;i<buf_size;i++){
        printf("%#x ", buf[i]);
    }
    printf("\n");
}

int fd1;
ofstream ofs;
bool is_write_csv;

const int MAX_SPEED = 4000; // mm/s
void serial_callback(const geometry_msgs::Twist vel){

    cout << "x:" << vel.linear.x;
    cout << ",th:" << vel.angular.z << endl;

    float vel_x = vel.linear.x; // m/s
    float vel_y = vel.linear.y;
    float ang_z = vel.angular.z; // rad/s

    int d = 100;

    float v_r = ( vel_x + d * ang_z ) * 1000; // mm/s
    float v_l = ( vel_x - d * ang_z ) * 1000; // mm/s

    if(abs(v_r) > MAX_SPEED || abs(v_l) > MAX_SPEED){
        // todo ROSERROR
        return;
    }

    uint16_t rspd = static_cast<uint16_t>((v_r / MAX_SPEED) * 127);
    uint16_t lspd = static_cast<uint16_t>((v_l / MAX_SPEED) * 127);

    // send
    //uint8_t sendBuf[] = {0x96, 0x46, rspd, lspd, 0};
    uint8_t sendBuf[] = {0x96, 0x47, static_cast<uint8_t>(rspd>>8), static_cast<uint8_t>(rspd), static_cast<uint8_t>(lspd>>8), static_cast<uint8_t>(lspd), 0};

    int bufSize = sizeof(sendBuf) / sizeof(sendBuf[0]);
    // sendBuf[bufSize-1] = calc_checksum(sendBuf, bufSize);

    uint8_t c_sum = calc_checksum(sendBuf, bufSize);
    sendBuf[bufSize-1] = c_sum;

    // cout << "rspd:" << rspd;
    // cout << ",lspd:" << lspd;
    // cout << ",c_sum:" << c_sum << endl;

    printf("rspd: %#x ", rspd);
    printf("lspd: %#x ", lspd);
    printf("\n");

    printf("write:");
    buf_print(sendBuf, bufSize);
    int rec=serial_write(fd1, sendBuf, sizeof(sendBuf));

    usleep(50000);

    if(rec>=0){
        uint8_t retbuf[64]={0};
        int recv_data=serial_read(fd1, retbuf, sizeof(retbuf));
        printf("recv %d \n", recv_data);

        printf("recv:");
        buf_print(retbuf, recv_data);
    }

    if(is_write_csv){
        char date[64];
        ros::Time t = ros::Time::now();
        time_t t_ = t.toSec();
        strftime(date, sizeof(date), "%H%M%S", localtime(&t_));
        char output_date[32];
        t_ = t.toNSec();
        snprintf(output_date, 32, "%s.%03d", date, static_cast<int>(t_)/1000000);

        ofs << output_date << ",";
        ofs << float(vel_x) << ",";
        ofs << float(vel_y) << ",";
        ofs << float(ang_z) << ",";
        ofs << int(rspd) << ",";
        ofs << int(lspd) << ",";
        ofs << static_cast<int>(c_sum) << ",";
        ofs << ",";
        for(auto b:sendBuf){
            ofs << static_cast<int>(b) << ",";
        }
        ofs << endl;
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_motordriver");
    ros::NodeHandle n;

    ros::Time current_time;
    current_time = ros::Time::now();

    ros::NodeHandle nh_private("~");

    string device_name;
    string twist_frame;
    string base_frame;

    bool r_dir;
    bool l_dir;

    string csv_name;

    nh_private.param<string>("device_name", device_name, "/dev/ttyACM0");
    nh_private.param<string>("twist_frame", twist_frame, "/cmd_vel");
    nh_private.param<string>("base_frame", base_frame, "base_footprint");

    nh_private.param<bool>("r_dir", r_dir, true);
    nh_private.param<bool>("l_dir", l_dir, true);

    nh_private.param<bool>("csv", is_write_csv, true);
    nh_private.param<string>("csv_name", csv_name, "");
    if(is_write_csv){
        if(csv_name == "")
        {
            char date[64];
            time_t t = ros::Time::now().sec;
            strftime(date, sizeof(date), "motor_%Y%m%d%a_%H%M%S.csv", localtime(&t));
            csv_name = date;
            cout << "motor Create CSV" << endl;
            cout << date << endl;
            cout << endl;
        }
        ofs = ofstream(csv_name);
    }
    else{
        cout << "motor No debug" << endl << endl;
    }


    //Subscriber
    ros::Subscriber serial_sub = n.subscribe(twist_frame, 10, serial_callback); 

    fd1=open_serial(device_name.c_str());

    if(fd1<0){
        ROS_ERROR("Serial Fail: cound not open %s", device_name.c_str());
        printf("Serial Fail\n");
        // ros::shutdown();
    }

    uint8_t state1 = 0b01100001;
    if(!r_dir){
        state1 ^= 0b01000000;
    }
    if(!l_dir){
        state1 ^= 0b00100000;
    }
    // set param
    uint8_t zeroBuf[] = {0x96, 0x41, state1, 0x00};
    int rec=serial_write(fd1, zeroBuf, sizeof(zeroBuf));
    // printf("send:%#x %#x\n",zeroBuf[0], zeroBuf[1]);
    if(rec>=0){
        uint8_t retbuf[64]={0};
        int recv_data=serial_read(fd1, retbuf, sizeof(retbuf));
        printf("recv %d \n", recv_data);
    }



    ros::spin();
     
    return 0;
}
