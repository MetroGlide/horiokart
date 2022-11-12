#include "horiokart2021_sensors/serial_odometry.hpp"

using namespace std;
using namespace horiokart2021_sensors;


SerialOdometry::SerialOdometry(string device_name, int read_sleep_usec)
:device_name(device_name), serial(device_name), sleep_usec(read_sleep_usec)
{
    if(!serial.is_open_serial){
        printf("Serial Fail: cound not open %s", device_name.c_str());
    }
}

// OdometryError SerialOdometry::checkError(std::vector<uint8_t> ret)
// {
//     if(ret.size() != GET_ODOM_RET_SIZE){
//         return OdometryError::ReceiveSizeError; // 受信データサイズerror
//     }

//     if(ret[0] != 0x24 || ret[1] != 0x75){
//         return OdometryError::InvalidHeader; // 返り値変
//     }

//     uint8_t checksum = serial.calc_checksum(vector<uint8_t>(ret.begin(), ret.end()-1));
//     if(checksum != ret.back())
//     {
//         return OdometryError::ChecksumError; // checksum error
//     }

//     // finally
//     return OdometryError::NoError;
// }

// OdometryData SerialOdometry::parse(std::vector<uint8_t> ret)
// {
//     OdometryData odom;

//     odom.x = double(((int32_t)ret[2]) << 24 | ((int32_t)ret[3]) << 16 | ((int32_t)ret[4]) << 8 | ((int32_t)ret[5])) / 1000;
//     odom.y = double(((int32_t)ret[6]) << 24 | ((int32_t)ret[7]) << 16 | ((int32_t)ret[8]) << 8 | ret[9]) / 1000;
//     odom.th = double(((uint16_t)ret[10]) << 8 | (uint16_t)ret[11]) / 10000;

//     odom.vx = double(int16_t(((uint16_t)ret[12]) << 8 | (uint16_t)ret[13])) / 1000;
//     odom.vy = double(int16_t(((uint16_t)ret[14]) << 8 | (uint16_t)ret[15])) / 1000;
//     odom.vth = double(int16_t(((uint16_t)ret[16]) << 8 | (uint16_t)ret[17])) / 10000;

//     return odom;
// }

OdometryData SerialOdometry::decode(std::vector<uint8_t> ret)
{
    OdometryData odom;
    odom.x = static_cast<double>(
        static_cast<int32_t>(serial.decode<uint32_t>(vector<uint8_t>{&ret[2], &ret[2]+4}))
     ) / 1000;
    odom.y = static_cast<double>(
        static_cast<int32_t>(serial.decode<uint32_t>(vector<uint8_t>{&ret[6], &ret[6]+4}))
     ) / 1000;
    odom.th = static_cast<double>(
        static_cast<uint16_t>(serial.decode<uint16_t>(vector<uint8_t>{&ret[10], &ret[10]+2}))
     ) / 10000;

    odom.vx = static_cast<double>(
        static_cast<int16_t>(serial.decode<uint16_t>(vector<uint8_t>{&ret[12], &ret[12]+2}))
     ) / 1000;
    odom.vy = static_cast<double>(
        static_cast<int16_t>(serial.decode<uint16_t>(vector<uint8_t>{&ret[14], &ret[14]+2}))
     ) / 1000;
    odom.vth = static_cast<double>(
        static_cast<int16_t>(serial.decode<uint16_t>(vector<uint8_t>{&ret[16], &ret[16]+2}))
     ) / 10000;

    odom.raw = ret;

    return odom;
}

OdometryData SerialOdometry::getData()
{
    OdometryData odom;
    SerialError e;

    // write
    int rec=serial.serial_write(OdomBuf, true, true);

    if(rec<=0){
        e = SerialError::WriteError;
        odom.error = e;
        return odom;
    }

    // read
    vector<uint8_t> retbuf = serial.serial_read(sleep_usec);
    odom.raw = retbuf;

    e = serial.checkError(retbuf, OdomBuf, GET_ODOM_RET_SIZE);
    if(e != SerialError::NoError)
    {
        odom.error = e;
        return odom;
    }

    // parse
    odom = decode(retbuf);
    odom.error = e;

    return odom;
}

SerialError SerialOdometry::sendZeroReset(int retry)
{
    for (int i = 0; i < retry;i++)
    {
        vector<uint8_t> ret = serial.serial_readwrite(ZeroBuf, sleep_usec);
        
        if(ret.size() == 3)
        {
            // todo check value
            return SerialError::NoError;
        }
    }
    return SerialError::OtherError;
}

bool SerialOdometry::isAlive()
{
    return serial.is_open_serial;
}
