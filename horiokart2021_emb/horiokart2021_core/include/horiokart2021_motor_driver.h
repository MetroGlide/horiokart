
#ifndef HORIOKART2021_MOTOR_DRIVER_H_
#define HORIOKART2021_MOTOR_DRIVER_H_

#include <stdint.h>

class HorioKart2021MotorDriver
{
 public:
  HorioKart2021MotorDriver();
  ~HorioKart2021MotorDriver();

  bool init();
  void close(void);

  bool readEncoder(int32_t &left_value, int32_t &right_value);
  bool writeVelocity(int64_t left_value, int64_t right_value);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

};

#endif