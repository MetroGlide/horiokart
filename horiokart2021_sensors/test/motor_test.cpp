#include "horiokart2021_sensors/serial_motordriver.hpp"

#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace horiokart2021_sensors;


const int TIME_TO_SLEEP = 10000; // usec

ostringstream ss;

SerialMotorDriver motor;


void init()
{
    motor = SerialMotorDriver("/dev/ttyHoriokart-motordriver", TIME_TO_SLEEP);
}

void print(vector<uint8_t> buf)
{
    for(auto b:buf)
    {
        printf("%#x ", b);
    }
    printf("\n");
}

int main()
{
    init();

    cout << endl << "serial motor test" << endl;

    SetSpeedRequest req;
    vector<uint8_t> m;

    req = SetSpeedRequest();
    req.rightWheelSpeed = 32;
    req.leftWheelSpeed = 25;

    m = motor.encode(req);
    print(m);

    req = SetSpeedRequest();
    req.rightWheelSpeed = -32;
    req.leftWheelSpeed = -25;

    m = motor.encode(req);
    print(m);

}
