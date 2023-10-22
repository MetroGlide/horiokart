#include "horiokart_drivers/base/serial_communicator2.hpp"

using namespace std;
using namespace horiokart_drivers;

SerialCommunicator2::SerialCommunicator2(string device_name)
    : device_name_(device_name)
{
    open_serial(device_name);
}

void SerialCommunicator2::open_serial(string device_name)
{

    // if (isOpened()) close();
    // serial_fd = ::open(portname, O_RDWR | O_NOCTTY | O_NDELAY);

    int serial_fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_fd == -1)
    {
        this->is_open_serial_ = false;
        printf("serial open failed...\n");
        return;
    }

    speed_t BAUDRATE = B57600;

#if !defined(MG_GNUC)
    // for standard UNIX
    struct termios options, oldopt;
    tcgetattr(serial_fd, &oldopt);
    bzero(&options, sizeof(struct termios));

    // enable rx and tx
    options.c_cflag |= (CLOCAL | CREAD);

    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    options.c_cflag &= ~PARENB;  // no checkbit
    options.c_cflag &= ~CSTOPB;  // 1bit stop bit
    options.c_cflag &= ~CRTSCTS; // no flow control

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; /* Select 8 data bits */

#ifdef CNEW_RTSCTS
    options.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
#endif

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control

    // raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // raw output mode
    options.c_oflag &= ~OPOST;

    if (tcsetattr(serial_fd, TCSANOW, &options))
    {
        // close();
        return;
    }

#else

    // using Linux extension ...
    struct termios2 tio;

    ioctl(serial_fd, TCGETS2, &tio);
    bzero(&tio, sizeof(struct termios2));

    tio.c_cflag = BOTHER;
    tio.c_cflag |= (CLOCAL | CREAD | CS8); // 8 bit no hardware handshake

    tio.c_cflag &= ~CSTOPB;  // 1 stop bit
    tio.c_cflag &= ~CRTSCTS; // No CTS
    tio.c_cflag &= ~PARENB;  // No Parity

#ifdef CNEW_RTSCTS
    tio.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
#endif

    tio.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control

    tio.c_cc[VMIN] = 0;  // min chars to read
    tio.c_cc[VTIME] = 0; // time in 1/10th sec wait

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // raw output mode
    tio.c_oflag &= ~OPOST;

    // tio.c_ispeed = baudrate;
    // tio.c_ospeed = baudrate;
    tio.c_ispeed = BAUDRATE;
    tio.c_ospeed = BAUDRATE;

    ioctl(serial_fd, TCSETS2, &tio);

#endif

    tcflush(serial_fd, TCIFLUSH);

    this->fd1_ = serial_fd;
    if (fcntl(serial_fd, F_SETFL, FNDELAY))
    {
        close(this->fd1_);
        this->is_open_serial_ = false;
    }
    else
    {
        this->is_open_serial_ = true;
    }
}

void SerialCommunicator2::reset_serial()
{
    if (this->is_open_serial_)
    {
        close(this->fd1_);
    }
    this->open_serial(this->device_name_);
}

uint8_t SerialCommunicator2::calc_checksum(vector<uint8_t> buf)
{
    uint8_t sum = 0;

    for (const auto &b : buf)
    {
        sum += b;
    }
    return sum;
}
SerialError SerialCommunicator2::check_error(std::vector<uint8_t> ret, std::vector<uint8_t> header, int size)
{
    if (ret.size() != size)
    {
        return SerialError::RECEIVE_SIZE_ERROR; // 受信データサイズerror
    }

    for (int i = 0; i < header.size(); i++)
    {
        if (ret[i] != header[i])
        {
            return SerialError::INVALID_HEADER; // 返り値変
        }
    }

    uint8_t checksum = calc_checksum(vector<uint8_t>(ret.begin(), ret.end() - 1));
    if (checksum != ret.back())
    {
        return SerialError::CHECKSUM_ERROR; // checksum error
    }

    // finally
    return SerialError::NO_ERROR;
}

int SerialCommunicator2::serial_write(vector<uint8_t> write_buf, bool input_flush, bool output_flush)
{
    if (!is_open_serial_)
        return -1;

    if (output_flush)
    {
        if (tcflush(this->fd1_, TCOFLUSH) < 0)
            printf("output flush error");
    }
    if (input_flush)
    {
        if (tcflush(this->fd1_, TCIFLUSH) < 0)
            printf("input flush error");
    }

    return write(this->fd1_, &write_buf[0], sizeof(write_buf[0]) * write_buf.size());
}
vector<uint8_t> SerialCommunicator2::serial_read(int sleep_usec)
{
    if (!is_open_serial_)
        return vector<uint8_t>();

    usleep(sleep_usec);

    uint8_t retbuf[64] = {0};
    int ret = read(this->fd1_, retbuf, sizeof(retbuf));

    return vector<uint8_t>(retbuf, retbuf + ret);
}
vector<uint8_t> SerialCommunicator2::serial_readwrite(vector<uint8_t> write_buf, int sleep_usec, bool input_flush, bool output_flush)
{
    this->serial_write(write_buf, input_flush, output_flush);
    return this->serial_read(sleep_usec);
}

string SerialCommunicator2::format_hex(vector<uint8_t> buf)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (const auto &b : buf)
    {
        ss << "0x" << std::setw(2) << static_cast<int>(b) << " ";
    }
    return ss.str();
}
