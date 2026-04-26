#pragma once

// #include "horiokart_drivers/base/serial_communicator.hpp"

#include <string>
#include <iostream>
#include <iomanip>
#include <unistd.h>

#include <fstream>
#include <vector>
#include <algorithm>
#include <time.h>

#include <fcntl.h>
#include <string.h> // for bzero()

// #define MG_GNUC
//__GNUC__
#if defined(MG_GNUC)
// for Linux extension
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
extern "C" int tcflush(int fildes, int queue_selector);
#else
// for other standard UNIX
#include <termios.h>
#include <sys/ioctl.h>

#endif

namespace horiokart_drivers
{
    enum class SerialError
    {
        NO_ERROR,
        WRITE_ERROR,
        RECEIVE_SIZE_ERROR,
        INVALID_HEADER,
        CHECKSUM_ERROR,
        OTHER_ERROR
    };
    const std::array<std::string, 6> SerialErrorStrings = {
        "NO_ERROR",
        "WRITE_ERROR",
        "RECEIVE_SIZE_ERROR",
        "INVALID_HEADER",
        "CHECKSUM_ERROR",
        "OTHER_ERROR"};

    class SerialCommunicator2
    {
    private:
        int fd1_;
        std::string device_name_;

        void open_serial(std::string device_name);

    public:
        bool is_open_serial_ = false;

        SerialCommunicator2(){};
        SerialCommunicator2(std::string device_name);

        void reset_serial();

        int serial_write(std::vector<uint8_t> buf, bool input_flush = true, bool output_flush = true);
        std::vector<uint8_t> serial_read(int sleep_usec = 5000);
        std::vector<uint8_t> serial_readwrite(std::vector<uint8_t> buf, int sleep_usec = 5000, bool input_flush = true, bool output_flush = true);

        uint8_t calc_checksum(std::vector<uint8_t>);

        SerialError check_error(std::vector<uint8_t> ret, std::vector<uint8_t> header, int size);

        static std::string format_hex(std::vector<uint8_t> buf);

        // template method implementation
        template <typename T>
        std::vector<uint8_t> encode(T data, bool big = true)
        {
            size_t s = sizeof(T);
            std::vector<uint8_t> ret(s);

            for (int i = 0; i < s; i++)
            {
                ret[i] = static_cast<uint8_t>(data >> (8 * (s - i - 1)));
            }

            return ret;
        }

        template <typename T>
        T decode(std::vector<uint8_t> buf, bool big = true)
        {
            if (buf.size() < 2)
            {
                // Exception
            }
            reverse(buf.begin(), buf.end());

            T ret = static_cast<T>(buf[0]);
            for (int i = 1; i < buf.size(); i++)
            {
                ret |= static_cast<T>(buf[i]) << 8 * i;
            }
            return ret;
        }
    };
}
