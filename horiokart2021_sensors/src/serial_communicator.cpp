#include "horiokart2021_sensors/serial_communicator.hpp"

using namespace std;
using namespace horiokart2021_sensors;

SerialCommunicator::SerialCommunicator(string device_name)
:device_name(device_name)
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
SerialError SerialCommunicator::checkError(std::vector<uint8_t> ret, std::vector<uint8_t> header, int size)
{
    if(ret.size() != size){
        return SerialError::ReceiveSizeError; // 受信データサイズerror
    }

    for (int i = 0; i < header.size();i++)
    {
        if(ret[i] != header[i])
        {
            return SerialError::InvalidHeader; // 返り値変
        }
    }

    uint8_t checksum = calc_checksum(vector<uint8_t>(ret.begin(), ret.end()-1));
    if(checksum != ret.back())
    {
        return SerialError::ChecksumError; // checksum error
    }

    // finally
    return SerialError::NoError;
}

int SerialCommunicator::serial_write(vector<uint8_t> write_buf, bool input_flush, bool output_flush)
{
    if(!is_open_serial) return -1;

    if (output_flush)
    {
        if (tcflush(this->fd1, TCOFLUSH) < 0) printf("output flush error");
    }
    if (input_flush)
    {
        if (tcflush(this->fd1, TCIFLUSH) < 0) printf("input flush error");
    }

    return write(this->fd1, &write_buf[0], sizeof(write_buf[0]) * write_buf.size());
}
vector<uint8_t> SerialCommunicator::serial_read(int sleep_usec)
{
    if(!is_open_serial) return vector<uint8_t>();

    usleep(sleep_usec);

    uint8_t retbuf[64]={0};
    int ret = read(this->fd1, retbuf, sizeof(retbuf));

    return vector<uint8_t>(retbuf, retbuf + ret);
}
vector<uint8_t> SerialCommunicator::serial_readwrite(vector<uint8_t> write_buf, int sleep_usec, bool input_flush, bool output_flush)
{
    this->serial_write(write_buf, input_flush, output_flush);
    return this->serial_read(sleep_usec);
}
