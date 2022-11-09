#include "horiokart2021_sensors/serial_odometry.hpp"

#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace horiokart2021_sensors;


const int TIME_TO_SLEEP = 10000; // usec

ostringstream ss;

SerialOdometry odometry;


void init()
{
    odometry = SerialOdometry("/dev/ttyHoriokart-odom", TIME_TO_SLEEP);
}

void printOdomData(OdometryData o)
{
    cout << "-----" << endl;
    cout << "x: " << o.x << endl;
    cout << "y: " << o.y << endl;
    cout << "th: " << o.th << endl;
    cout << "vx: " << o.vx << endl;
    cout << "vy: " << o.vy << endl;
    cout << "vth: " << o.vth << endl;
}

int main()
{
    init();

    cout << endl << "serial odom test" << endl;

    // x:100.5m, y:200.6m th:1.5rad vx:20.3m/s vy:14.7m/s vth:2.1rad/s
    vector<uint8_t> buf1{0x24, 0x75,
                         0x00, 0x01, 0x88, 0x94,
                         0x00, 0x03, 0x0F, 0x98,
                         0x3A, 0x98,
                         0x4F, 0x4C,
                         0x39, 0x6C,
                         0x52, 0x08,
                         0x00};
    OdometryData o = odometry.parse(buf1);
    printOdomData(o);

    // x:-100.5m, y:-200.6m th:-1.5rad vx:-20.3m/s vy:-14.7m/s vth:-2.1rad/s
    vector<uint8_t> buf2{0x24, 0x75,
                         0xFF, 0xFE, 0x77, 0x6C,
                         0xFF, 0xFC, 0xF0, 0x68,
                         0x3A, 0x98,
                         0xB0, 0xB4,
                         0xC6, 0x94,
                         0xAD, 0xF8,
                         0x00};
    o = odometry.parse(buf2);
    printOdomData(o);
}
