#include "horiokart2021_sensors/serial_communicator.hpp"

using namespace std;
using namespace horiokart2021_sensors;

SerialCommunicator::SerialCommunicator(string device_name, int sleep_time)
:device_name(device_name),time_to_sleep(sleep_time)
{
    open_serial(device_name);
}

void SerialCommunicator::open_serial(string device_name){
    // int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    int fd1=open(device_name.c_str(), O_RDWR | O_NOCTTY );

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

    this->fd1 = fd1;

    if(this->fd1 < 0){
        this->is_open_serial = false;
    }
    else{
        this->is_open_serial = true;
    }
}

uint8_t SerialCommunicator::calc_checksum(vector<uint8_t> buf){
    uint8_t sum = 0;
    
    for(const auto& b: buf)
    {
        sum += b;
    }
    return sum;
}

int SerialCommunicator::serial_write(vector<uint8_t> write_buf)
{
    if(is_open_serial){
        int ret = tcflush(this->fd1, TCOFLUSH);
        if(ret < 0){
            printf("flush error");
        }
        return write(this->fd1, &write_buf[0], sizeof(write_buf[0]) * write_buf.size());
    }
    else{
        return -1;
    }
}
vector<uint8_t> SerialCommunicator::serial_read()
{
    if(is_open_serial){
        // int ret = tcflush(this->fd1, TCIFLUSH);
        // if(ret < 0){
        //     printf("flush error");
        // }

        uint8_t retbuf[64]={0};
        int ret = read(this->fd1, retbuf, sizeof(retbuf));

        return vector<uint8_t>(retbuf, retbuf + ret);
    }
    else{
        return vector<uint8_t>();
    }
}
vector<uint8_t> SerialCommunicator::serial_readwrite(vector<uint8_t> write_buf)
{
    int rec = this->serial_write(write_buf);

    if(rec>=0)
    {
        usleep(time_to_sleep);
        return this->serial_read();
    }
    else
    {
        return vector<uint8_t>();
    }

}
