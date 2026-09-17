#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "bno_config.h"

/* ===== I2C Address ===== */

#define BNO_ADDR_ALT        0x28U
#define BNO_ADDR            0x29U

/* ===== Registers ===== */

#define BNO_CHIP_ID         0x00U
#define BNO_PAGE_ID         0x07U

#define BNO_ACC_DATA        0x08U
#define BNO_MAG_DATA        0x0EU
#define BNO_GYR_DATA        0x14U
#define BNO_EUL_DATA        0x1AU
#define BNO_QUA_DATA        0x20U
#define BNO_LIA_DATA        0x28U
#define BNO_GRV_DATA        0x2EU

#define BNO_TEMP            0x34U
#define BNO_CALIB_STAT      0x35U

#define BNO_UNIT_SEL        0x3BU
#define BNO_OPR_MODE        0x3DU
#define BNO_PWR_MODE        0x3EU
#define BNO_SYS_TRIGGER     0x3FU

#define BNO_CHIP_ID_VALUE   0xA0U

/* ===== Scale ===== */

#define BNO_ACC_SCALE       100.0f
#define BNO_GYR_SCALE       16.0f
#define BNO_MAG_SCALE       16.0f
#define BNO_EUL_SCALE       16.0f
#define BNO_QUA_SCALE       16384.0f

/* ===== Error ===== */

typedef enum
{
    BNO_OK = 0,
    BNO_ERR_NULL_PTR,
    BNO_ERR_I2C,
    BNO_ERR_WRONG_CHIP_ID,
    BNO_ERR_BAD_ARG,
    BNO_ERR_NOT_INITIALIZED

} error_bno;

/* ===== Operation Mode ===== */

typedef enum
{
    BNO_MODE_CONFIG         = 0x00,
    BNO_MODE_ACCONLY        = 0x01,
    BNO_MODE_MAGONLY        = 0x02,
    BNO_MODE_GYRONLY        = 0x03,
    BNO_MODE_ACCMAG         = 0x04,
    BNO_MODE_ACCGYRO        = 0x05,
    BNO_MODE_MAGGYRO        = 0x06,
    BNO_MODE_AMG            = 0x07,

    BNO_MODE_IMU            = 0x08,
    BNO_MODE_COMPASS        = 0x09,
    BNO_MODE_M4G            = 0x0A,
    BNO_MODE_NDOF_FMC_OFF   = 0x0B,
    BNO_MODE_NDOF           = 0x0C

} bno055_opmode_t;

/* ===== Unit ===== */

typedef enum
{
    BNO_TEMP_UNIT_C = 0,
    BNO_TEMP_UNIT_F = 1

} bno055_temp_unitsel_t;

typedef enum
{
    BNO_GYR_UNIT_DPS = 0,
    BNO_GYR_UNIT_RPS = 1

} bno055_gyr_unitsel_t;

typedef enum
{
    BNO_ACC_UNITSEL_M_S2 = 0,
    BNO_ACC_UNITSEL_MG   = 1

} bno055_acc_unitsel_t;

typedef enum
{
    BNO_EUL_UNIT_DEG = 0,
    BNO_EUL_UNIT_RAD = 1

} bno055_eul_unitsel_t;

/* ===== Data Types ===== */

typedef struct
{
    float x;
    float y;
    float z;

} bno055_vec3_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;

} bno055_vec4_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;

} bno055_euler_t;

typedef struct
{
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;

} bno055_calib_t;

/* ===== Device ===== */

typedef struct
{
    I2C_HandleTypeDef *i2c;

    uint8_t addr;
    bno055_opmode_t mode;

    uint8_t initialized;

} bno055_t;


/*
 * Project-wide BNO055 instance.
 *
 * main.c / cdc_protocol.c からこれを共用する。
 */
extern bno055_t bno;


/* ===== API ===== */

error_bno bno055_init(bno055_t *imu);

error_bno bno055_set_opmode(
    bno055_t *imu,
    bno055_opmode_t mode
);

error_bno bno055_set_unit(
    bno055_t *imu,
    bno055_temp_unitsel_t temp,
    bno055_gyr_unitsel_t gyro,
    bno055_acc_unitsel_t accel,
    bno055_eul_unitsel_t euler
);

error_bno bno055_euler(
    bno055_t *imu,
    bno055_euler_t *data
);

error_bno bno055_quaternion(
    bno055_t *imu,
    bno055_vec4_t *data
);

error_bno bno055_acc(
    bno055_t *imu,
    bno055_vec3_t *data
);

error_bno bno055_gyro(
    bno055_t *imu,
    bno055_vec3_t *data
);

error_bno bno055_mag(
    bno055_t *imu,
    bno055_vec3_t *data
);

error_bno bno055_linear_acc(
    bno055_t *imu,
    bno055_vec3_t *data
);

error_bno bno055_gravity(
    bno055_t *imu,
    bno055_vec3_t *data
);

error_bno bno055_temperature(
    bno055_t *imu,
    int8_t *temperature
);

error_bno bno055_calibration(
    bno055_t *imu,
    bno055_calib_t *calib
);

const char *bno055_err_str(error_bno err);

#endif