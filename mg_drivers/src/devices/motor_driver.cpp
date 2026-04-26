#include "horiokart_drivers/devices/motor_driver.hpp"

using namespace std;
using namespace horiokart_drivers;

MotorDriver::MotorDriver(string device_name)
    : device_name_(device_name),
      serial_(device_name)
{
    if (!serial_.is_open_serial_)
    {
        printf("Serial Fail: cound not open %s", device_name.c_str());
    }
}

MotorDriverResponse MotorDriver::send_speed_command(SpeedParameter param)
{
    MotorDriverResponse res;
    SerialError e;

    vector<uint8_t> send_buf = this->encode(param);
    int rec = serial_.serial_write(send_buf, true, true);
    if (rec <= 0)
    {
        e = SerialError::WRITE_ERROR;
        res.error = e;
        return res;
    }

    // read
    vector<uint8_t> retbuf = serial_.serial_read(sleep_usec_);
    res.raw = retbuf;

    e = serial_.check_error(retbuf, send_speed_command_header_, receive_speed_command_size_);
    if (e != SerialError::NO_ERROR)
    {
        res.error = e;
        return res;
    }

    return res;
}

vector<uint8_t> MotorDriver::encode(SpeedParameter param)
{
    // vector<uint8_t> ret(SET_SPEED_BUF_SIZE);
    vector<uint8_t> ret;

    // add base command
    ret.insert(ret.begin(), send_speed_command_header_.begin(), send_speed_command_header_.end());

    // add right wheel speed
    vector<uint8_t> r = serial_.encode(param.right_wheel_speed);
    ret.insert(ret.begin() + 2, r.begin(), r.end());

    // add left wheel speed
    vector<uint8_t> l = serial_.encode(param.left_wheel_speed);
    ret.insert(ret.begin() + 4, l.begin(), l.end());

    uint8_t checksum = serial_.calc_checksum(vector<uint8_t>(ret.begin(), ret.end()));

    ret.push_back(checksum);

    return ret;
}

bool MotorDriver::is_alive()
{
    return serial_.is_open_serial_;
}