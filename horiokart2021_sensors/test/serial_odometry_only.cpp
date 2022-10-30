#include "horiokart2021_sensors/serial_odometry.hpp"

#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace horiokart2021_sensors;

bool is_write_csv = true;
string csv_name;
ofstream ofs;

const int TIME_TO_SLEEP = 10000; // usec

ostringstream ss;

SerialOdometry odometry;
OdometryData currentData;


void init()
{
    if(is_write_csv){
        if(csv_name == "")
        {
            char date[64];
            time_t t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            strftime(date, sizeof(date), "odom_%Y%m%d%a_%H%M%S.csv", localtime(&t));
            csv_name = date;

            printf("Odom create CSV %s \n", csv_name.c_str());
        }
        ofs = ofstream(csv_name);
    }
    else{
        printf("Odom No CSV \n");
    }

    SerialOdometry odometry = SerialOdometry("/dev/ttyHoriokart-odom", TIME_TO_SLEEP);
}


void update_odom()
{
    currentData = odometry.getData();

    ss.str("");
    ss << "recv: ";
    for( const auto& b: currentData.raw)
    {
        ss << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b) << " ";

    }
    cout << ss.str() << endl;
    if (currentData.error != OdometryError::NoError)
    {
        cout << "ERROR code " << static_cast<int>(currentData.error) << endl;
    }
}

void write_log_csv()
{
    if(is_write_csv){

        chrono::system_clock::time_point tp = chrono::system_clock::now();

        ofs << chrono::duration_cast<chrono::milliseconds>(tp.time_since_epoch()).count() << ",";
        ofs << currentData.x << ",";
        ofs << currentData.y << ",";
        ofs << currentData.th << ",";
        ofs << currentData.x << ",";
        ofs << currentData.y << ",";
        ofs << currentData.th << ",";
        ofs << ",";
        ofs << static_cast<int>(currentData.error) << ",";
        ofs << ",";
        for(auto& b:currentData.raw){
            ofs << static_cast<int>(b) << ",";
        }
        ofs << endl;
    }

}

int main()
{
    init();

    cout << endl << "start serial odom" << endl;

    OdometryError e = odometry.sendZeroReset(3);
    if (e == OdometryError::NoError)
    {
        cout << "Odom zero reset" << endl;
    }
    else{
        cout << "Odom error zero reset!" << endl;
    }

    while (true){

        update_odom();
        write_log_csv();

        this_thread::sleep_for(chrono::milliseconds(1000));
    } 
}