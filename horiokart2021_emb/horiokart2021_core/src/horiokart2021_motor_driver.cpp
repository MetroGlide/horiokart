#include "../include/horiokart2021_motor_driver.h"

HorioKart2021MotorDriver::HorioKart2021MotorDriver()
{

}

HorioKart2021MotorDriver::~HorioKart2021MotorDriver()
{
  close();
}

bool HorioKart2021MotorDriver::init()
{
  // TODO 初期化処理
  return true;
}

void HorioKart2021MotorDriver::close(void)
{
  // TODO 後処理（必要であれば）
}

bool HorioKart2021MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  // エンコーダ値読み取り
  // パルス値生データ的なもの

  return true;
}

bool HorioKart2021MotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{

  return true;
}

bool HorioKart2021MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{

  return true;
}