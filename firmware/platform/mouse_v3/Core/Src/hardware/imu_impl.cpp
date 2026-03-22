#include "hardware/imu_impl.hpp"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;  // main.c

static constexpr float GYRO_VELOCITY_COMPENSATION = 1.2f;

static constexpr uint8_t IMU_ADDR = (0x68 << 1);  // 7-bit address, so shift left
static constexpr uint8_t IMU_WHO_AM_I = 0x67;

static constexpr uint8_t REG_WHO_AM_I = 0x75;
static constexpr uint8_t REG_PWR_MGMT0 = 0x1F;
static constexpr uint8_t REG_INT_CONFIG = 0x06;
static constexpr uint8_t REG_INT_SOURCE0 = 0x2B;
static constexpr uint8_t REG_GYRO_CONFIG0 = 0x20;
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x21;

static constexpr uint8_t REG_ACCEL_DATA_X1 = 0x0B;
static constexpr uint8_t REG_ACCEL_DATA_X0 = 0x0C;
static constexpr uint8_t REG_ACCEL_DATA_Y1 = 0x0D;
static constexpr uint8_t REG_ACCEL_DATA_Y0 = 0x0E;
static constexpr uint8_t REG_ACCEL_DATA_Z1 = 0x0F;
static constexpr uint8_t REG_ACCEL_DATA_Z0 = 0x10;

static constexpr uint8_t REG_GYRO_DATA_X1 = 0x11;
static constexpr uint8_t REG_GYRO_DATA_X0 = 0x12;
static constexpr uint8_t REG_GYRO_DATA_Y1 = 0x13;
static constexpr uint8_t REG_GYRO_DATA_Y0 = 0x14;
static constexpr uint8_t REG_GYRO_DATA_Z1 = 0x15;
static constexpr uint8_t REG_GYRO_DATA_Z0 = 0x16;

static constexpr uint32_t I2C_TIMEOUT = 100;

IMUImpl::IMUImpl(const Config& config) : m_config(config) {
  const auto fail_if = [](bool condition) {
    if (condition) {
      // ErrorManager::get().fatal_error(Error::IMU_CONFIG_FAIL);
    }
  };

  HAL_StatusTypeDef status;
  uint8_t buf[1];

  // Check WHO_AM_I register to verify I2C communication.
  status = read_register(REG_WHO_AM_I, buf, 1);
  fail_if(HAL_OK != status);
  fail_if(IMU_WHO_AM_I != buf[0]);

  const Config::Gyro& gyro_config = m_config.gyro;
  const Config::Accelerometer& accel_config = m_config.accel;

  // Configure gyro range and sample rate.

  const uint8_t gyro_range = uint8_t(gyro_config.range);
  const uint8_t gyro_sample_rate = uint8_t(gyro_config.sample_rate);

  buf[0] = gyro_range | gyro_sample_rate;
  status = write_register(REG_GYRO_CONFIG0, buf[0]);
  fail_if(HAL_OK != status);

  // Configure accelerometer range and sample rate.

  const uint8_t accel_range = uint8_t(accel_config.range);
  const uint8_t accel_sample_rate = uint8_t(accel_config.sample_rate);

  buf[0] = accel_range | accel_sample_rate;
  status = write_register(REG_ACCEL_CONFIG0, buf[0]);
  fail_if(HAL_OK != status);

  // Default to standby mode.

  set_standby(true);
}

void IMUImpl::publish_periodic_feedback() {
  using namespace feedback;

  m_feedback.publish<TopicSend::DRIVE_IMU_DATA>(m_data);
}

void IMUImpl::set_standby(bool on_standby) {
  using GyroMode = Config::Gyro::Mode;
  using AccelMode = Config::Accelerometer::Mode;

  if (m_standby == on_standby)
    return;
  m_standby = on_standby;

  HAL_StatusTypeDef status;

  GyroMode gyro_mode;
  AccelMode accel_mode;

  if (on_standby) {
    gyro_mode = GyroMode::STANDBY;
    accel_mode = AccelMode::STANDBY;
  } else {
    gyro_mode = m_config.gyro.mode;
    accel_mode = m_config.accel.mode;
  }

  const uint8_t value = uint8_t(gyro_mode) | uint8_t(accel_mode);

  status = write_register(REG_PWR_MGMT0, value);
  UNUSED(status);

  std::memset(&m_data, 0, sizeof(m_data));

  m_time_since_standby_timer->reset();
  m_time_since_standby_timer->start();

  for (Axis axis : {Axis::X, Axis::Y, Axis::Z}) {
    m_gyro_filters[axis].reset();
  }
}

HAL_StatusTypeDef IMUImpl::write_register(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};

  return HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR, data, 2, I2C_TIMEOUT);
}

HAL_StatusTypeDef IMUImpl::read_register(uint8_t reg, uint8_t* buf, uint8_t len) {
  assert_param(len > 0);
  assert_param(buf != nullptr);

  HAL_StatusTypeDef status;

  // Write register address.
  status = HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR, &reg, 1, I2C_TIMEOUT);
  if (status != HAL_OK)
    return status;

  // Read data.
  status = HAL_I2C_Master_Receive(&hi2c1, IMU_ADDR, buf, len, I2C_TIMEOUT);
  return status;
}

void IMUImpl::begin_read() {
  if (m_is_receiving)
    return;

  if (m_standby || m_time_since_standby_timer->get() < 5_ms) {
    return;
  }

  HAL_StatusTypeDef status;

  uint8_t buf[1] = {REG_ACCEL_DATA_X1};

  // Write register address.
  status = HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR, buf, 1, I2C_TIMEOUT);
  if (status != HAL_OK)
    return;

  // Read data.
  status = HAL_I2C_Master_Receive_DMA(&hi2c1, IMU_ADDR, m_data_raw, 12);
  if (status != HAL_OK)
    return;

  m_is_receiving = true;
}

void IMUImpl::read_complete_handler() {
  if (!m_is_receiving)
    return;

  units::standard_gravity_t accel_conversion;
  switch (m_config.accel.range) {
    case Config::Accelerometer::Range::_2_G:
      accel_conversion = (2_SG / INT16_MAX);
      break;
    case Config::Accelerometer::Range::_4_G:
      accel_conversion = (4_SG / INT16_MAX);
      break;
    case Config::Accelerometer::Range::_8_G:
      accel_conversion = (8_SG / INT16_MAX);
      break;
    case Config::Accelerometer::Range::_16_G:
    default:
      accel_conversion = (16_SG / INT16_MAX);
      break;
  }

  const int16_t accel_x = (m_data_raw[0] << 8) | m_data_raw[1];
  const int16_t accel_y = (m_data_raw[2] << 8) | m_data_raw[3];
  const int16_t accel_z = (m_data_raw[4] << 8) | m_data_raw[5];

  m_data.accel_data[Axis::X] = accel_x * accel_conversion;
  m_data.accel_data[Axis::Y] = accel_y * accel_conversion;
  m_data.accel_data[Axis::Z] = accel_z * accel_conversion;

  units::degrees_per_second_t gyro_conversion;
  switch (m_config.gyro.range) {
    case Config::Gyro::Range::_250_DPS:
      gyro_conversion = (250_deg_per_s / INT16_MAX);
      break;
    case Config::Gyro::Range::_500_DPS:
      gyro_conversion = (500_deg_per_s / INT16_MAX);
      break;
    case Config::Gyro::Range::_1000_DPS:
      gyro_conversion = (1000_deg_per_s / INT16_MAX);
      break;
    case Config::Gyro::Range::_2000_DPS:
    default:
      gyro_conversion = (2000_deg_per_s / INT16_MAX);
      break;
  }

  const int16_t gyro_x = (m_data_raw[6] << 8) | m_data_raw[7];
  const int16_t gyro_y = (m_data_raw[8] << 8) | m_data_raw[9];
  const int16_t gyro_z = (m_data_raw[10] << 8) | m_data_raw[11];

  m_data.gyro_data[Axis::X] = m_gyro_filters[Axis::X].calculate(gyro_x * gyro_conversion);
  m_data.gyro_data[Axis::Y] = m_gyro_filters[Axis::Y].calculate(gyro_y * gyro_conversion);
  m_data.gyro_data[Axis::Z] = m_gyro_filters[Axis::Z].calculate(gyro_z * gyro_conversion);

  m_is_receiving = false;
}

// I2C DMA interrupt callback.
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  assert_param(hi2c != nullptr);
  assert_param(hi2c == &hi2c1);
  UNUSED(hi2c);

  IMUImpl* imu = reinterpret_cast<IMUImpl*>(&get_platform_imu());
  imu->read_complete_handler();
}

IMUImpl& get_mouse_v3_imu() {
  static IMUImpl s_imu{IMUImpl::Config{}};
  return s_imu;
}

hardware::IMU& get_platform_imu() {
  return get_mouse_v3_imu();
}
