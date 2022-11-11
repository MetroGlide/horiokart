#include "horiokart2021_sensors/serial_motordriver.hpp"

using namespace std;
using namespace horiokart2021_sensors;


SerialMotorDriver::SerialMotorDriver(string device_name, int read_sleep_usec)
:device_name(device_name), serial(device_name), sleep_usec(read_sleep_usec)
{
    if(!serial.is_open_serial){
        printf("Serial Fail: cound not open %s", device_name.c_str());
    }
}


MotorDriverResponse SerialMotorDriver::setSpeedData(SetSpeedRequest data)
{
    MotorDriverResponse res;
    SerialError e;

    vector<uint8_t> sendBuf = encode(data);
    int rec = serial.serial_write(sendBuf, true, true);
    if(rec<=0){
        e = SerialError::WriteError;
        res.error = e;
        return res;
    }

    // read
    vector<uint8_t> retbuf = serial.serial_read(sleep_usec);
    res.raw = retbuf;

    e = serial.checkError(retbuf, SetSpeedBufBase, SET_SPEED_BUF_RET_SIZE);
    if(e != SerialError::NoError)
    {
        res.error = e;
        return res;
    }

    return res;
}

vector<uint8_t> SerialMotorDriver::encode(SetSpeedRequest data)
{
    // vector<uint8_t> ret(SET_SPEED_BUF_SIZE);
    vector<uint8_t> ret;

    // add base command
    ret.insert(ret.begin(), SetSpeedBufBase.begin(), SetSpeedBufBase.end());

    // add right wheel speed
    vector<uint8_t> r = serial.encode(data.rightWheelSpeed);
    ret.insert(ret.begin()+2, r.begin(), r.end());

    // add left wheel speed
    vector<uint8_t> l = serial.encode(data.leftWheelSpeed);
    ret.insert(ret.begin()+4, l.begin(), l.end());

    uint8_t checksum = serial.calc_checksum(vector<uint8_t>(ret.begin(), ret.end()-1));

    ret.push_back(checksum);

    return ret;
}

bool SerialMotorDriver::isAlive()
{
    return serial.is_open_serial;
}