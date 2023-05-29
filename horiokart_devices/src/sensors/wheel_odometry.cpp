#include "horiokart_devices/sensors/wheel_odometry.hpp"

using namespace std;
using namespace horiokart_devices;


WheelOdometry::WheelOdometry(string device_name)
    : device_name_(device_name),
      serial_(device_name)
{
    if (!serial_.is_open_serial_)
    {
        printf("Serial Fail: cound not open %s", device_name_.c_str());
    }
}

OdometryData WheelOdometry::decode(std::vector<uint8_t> ret)
{
    OdometryData odom;
    odom.x = static_cast<double>(
                 static_cast<int32_t>(serial_.decode<uint32_t>(vector<uint8_t>{&ret[2], &ret[2] + 4}))) /
             1000;
    odom.y = static_cast<double>(
                 static_cast<int32_t>(serial_.decode<uint32_t>(vector<uint8_t>{&ret[6], &ret[6] + 4}))) /
             1000;
    odom.th = static_cast<double>(
                  static_cast<uint16_t>(serial_.decode<uint16_t>(vector<uint8_t>{&ret[10], &ret[10] + 2}))) /
              10000;

    odom.vx = static_cast<double>(
                  static_cast<int16_t>(serial_.decode<uint16_t>(vector<uint8_t>{&ret[12], &ret[12] + 2}))) /
              1000;
    odom.vy = static_cast<double>(
                  static_cast<int16_t>(serial_.decode<uint16_t>(vector<uint8_t>{&ret[14], &ret[14] + 2}))) /
              1000;
    odom.vth = static_cast<double>(
                   static_cast<int16_t>(serial_.decode<uint16_t>(vector<uint8_t>{&ret[16], &ret[16] + 2}))) /
               10000;

    odom.raw = ret;

    return odom;
}

OdometryData WheelOdometry::get_data()
{
    OdometryData odom;
    SerialError err;

    // write
    int rec = serial_.serial_write(odom_buf_, true, true);

    if (rec <= 0)
    {
        err = SerialError::WRITE_ERROR;
        odom.error = err;
        return odom;
    }

    // read
    vector<uint8_t> retbuf = serial_.serial_read(sleep_usec_);
    odom.raw = retbuf;

    err = serial_.check_error(retbuf, odom_buf_, get_odom_ret_size_);
    if (err != SerialError::NO_ERROR)
    {
        odom.error = err;
        return odom;
    }

    // decode
    odom = decode(retbuf);
    odom.error = err;

    return odom;
}

SerialError WheelOdometry::send_zero_reset(int retry)
{
    for (int i = 0; i < retry; i++)
    {
        vector<uint8_t> ret = serial_.serial_readwrite(zero_buf_, sleep_usec_);

        if (ret.size() == 3)
        {
            // todo check value
            return SerialError::NO_ERROR;
        }
    }
    return SerialError::OTHER_ERROR;
}

bool WheelOdometry::is_alive()
{
    return serial_.is_open_serial_;
}
