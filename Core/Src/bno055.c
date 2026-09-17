#include "bno055.h"


/* ===== Global Device ===== */

bno055_t bno = {0};


/* ===== Private ===== */

static error_bno bno055_read(
    bno055_t *imu,
    uint8_t reg,
    uint8_t *data,
    uint16_t len
)
{
    if (imu == NULL || imu->i2c == NULL || data == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        imu->i2c,
        (uint16_t)(imu->addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        BNO055_I2C_TIMEOUT_MS
    );

    if (status != HAL_OK)
    {
        return BNO_ERR_I2C;
    }

    return BNO_OK;
}


static error_bno bno055_write(
    bno055_t *imu,
    uint8_t reg,
    uint8_t value
)
{
    if (imu == NULL || imu->i2c == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(
        imu->i2c,
        (uint16_t)(imu->addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        BNO055_I2C_TIMEOUT_MS
    );

    if (status != HAL_OK)
    {
        return BNO_ERR_I2C;
    }

    return BNO_OK;
}


static int16_t make_int16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(
        ((uint16_t)msb << 8) |
        (uint16_t)lsb
    );
}


static error_bno read_vec3(
    bno055_t *imu,
    uint8_t reg,
    float scale,
    bno055_vec3_t *data
)
{
    if (data == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (!imu->initialized)
    {
        return BNO_ERR_NOT_INITIALIZED;
    }

    uint8_t raw[6];

    error_bno err = bno055_read(
        imu,
        reg,
        raw,
        sizeof(raw)
    );

    if (err != BNO_OK)
    {
        return err;
    }

    data->x = (float)make_int16(raw[0], raw[1]) / scale;
    data->y = (float)make_int16(raw[2], raw[3]) / scale;
    data->z = (float)make_int16(raw[4], raw[5]) / scale;

    return BNO_OK;
}


/* ===== Initialization ===== */

error_bno bno055_init(bno055_t *imu)
{
    if (imu == NULL || imu->i2c == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (
        imu->addr != BNO_ADDR &&
        imu->addr != BNO_ADDR_ALT
    )
    {
        return BNO_ERR_BAD_ARG;
    }

    if (imu->mode > BNO_MODE_NDOF)
    {
        return BNO_ERR_BAD_ARG;
    }

    imu->initialized = 0;

    bno055_opmode_t requested_mode = imu->mode;

    uint8_t chip_id = 0;


    /*
     * Sensor might still be booting when MCU starts.
     */
    error_bno err = bno055_read(
        imu,
        BNO_CHIP_ID,
        &chip_id,
        1
    );

    if (err != BNO_OK || chip_id != BNO_CHIP_ID_VALUE)
    {
        HAL_Delay(BNO055_RESET_DELAY_MS);

        err = bno055_read(
            imu,
            BNO_CHIP_ID,
            &chip_id,
            1
        );

        if (err != BNO_OK)
        {
            return err;
        }
    }

    if (chip_id != BNO_CHIP_ID_VALUE)
    {
        return BNO_ERR_WRONG_CHIP_ID;
    }


    /*
     * Enter CONFIG mode
     */
    err = bno055_set_opmode(
        imu,
        BNO_MODE_CONFIG
    );

    if (err != BNO_OK)
    {
        return err;
    }


    /*
     * Software reset
     */
    err = bno055_write(
        imu,
        BNO_SYS_TRIGGER,
        0x20U
    );

    if (err != BNO_OK)
    {
        return err;
    }

    HAL_Delay(BNO055_RESET_DELAY_MS);


    /*
     * Wait until BNO055 comes back.
     */
    for (
        uint32_t i = 0;
        i < BNO055_BOOT_RETRY_COUNT;
        i++
    )
    {
        chip_id = 0;

        err = bno055_read(
            imu,
            BNO_CHIP_ID,
            &chip_id,
            1
        );

        if (
            err == BNO_OK &&
            chip_id == BNO_CHIP_ID_VALUE
        )
        {
            break;
        }

        HAL_Delay(BNO055_BOOT_RETRY_DELAY_MS);
    }

    if (chip_id != BNO_CHIP_ID_VALUE)
    {
        return BNO_ERR_WRONG_CHIP_ID;
    }


    /*
     * Normal power mode
     */
    err = bno055_write(
        imu,
        BNO_PWR_MODE,
        0x00U
    );

    if (err != BNO_OK)
    {
        return err;
    }

    HAL_Delay(10);


    /*
     * Page 0
     */
    err = bno055_write(
        imu,
        BNO_PAGE_ID,
        0x00U
    );

    if (err != BNO_OK)
    {
        return err;
    }


    /*
     * Internal oscillator
     */
    err = bno055_write(
        imu,
        BNO_SYS_TRIGGER,
        0x00U
    );

    if (err != BNO_OK)
    {
        return err;
    }


    /*
     * Default units:
     *
     * temperature : C
     * gyro        : deg/s
     * accel       : m/s^2
     * euler       : degree
     */
    err = bno055_write(
        imu,
        BNO_UNIT_SEL,
        0x00U
    );

    if (err != BNO_OK)
    {
        return err;
    }


    /*
     * Requested operating mode
     */
    err = bno055_set_opmode(
        imu,
        requested_mode
    );

    if (err != BNO_OK)
    {
        return err;
    }

    imu->initialized = 1;

    return BNO_OK;
}


/* ===== Mode ===== */

error_bno bno055_set_opmode(
    bno055_t *imu,
    bno055_opmode_t mode
)
{
    if (imu == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (mode > BNO_MODE_NDOF)
    {
        return BNO_ERR_BAD_ARG;
    }

    error_bno err = bno055_write(
        imu,
        BNO_OPR_MODE,
        (uint8_t)mode
    );

    if (err != BNO_OK)
    {
        return err;
    }

    HAL_Delay(BNO055_MODE_DELAY_MS);

    imu->mode = mode;

    return BNO_OK;
}


/* ===== Units ===== */

error_bno bno055_set_unit(
    bno055_t *imu,
    bno055_temp_unitsel_t temp,
    bno055_gyr_unitsel_t gyro,
    bno055_acc_unitsel_t accel,
    bno055_eul_unitsel_t euler
)
{
    if (imu == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    bno055_opmode_t old_mode = imu->mode;

    error_bno err;


    if (old_mode != BNO_MODE_CONFIG)
    {
        err = bno055_set_opmode(
            imu,
            BNO_MODE_CONFIG
        );

        if (err != BNO_OK)
        {
            return err;
        }
    }


    uint8_t reg = 0;

    /*
     * UNIT_SEL
     *
     * bit0 ACC
     * bit1 GYR
     * bit2 EUL
     * bit4 TEMP
     */
    if (accel == BNO_ACC_UNITSEL_MG)
    {
        reg |= (1U << 0);
    }

    if (gyro == BNO_GYR_UNIT_RPS)
    {
        reg |= (1U << 1);
    }

    if (euler == BNO_EUL_UNIT_RAD)
    {
        reg |= (1U << 2);
    }

    if (temp == BNO_TEMP_UNIT_F)
    {
        reg |= (1U << 4);
    }


    err = bno055_write(
        imu,
        BNO_UNIT_SEL,
        reg
    );

    if (err != BNO_OK)
    {
        return err;
    }


    if (old_mode != BNO_MODE_CONFIG)
    {
        err = bno055_set_opmode(
            imu,
            old_mode
        );

        if (err != BNO_OK)
        {
            return err;
        }
    }

    return BNO_OK;
}


/* ===== Euler ===== */

error_bno bno055_euler(
    bno055_t *imu,
    bno055_euler_t *data
)
{
    if (imu == NULL || data == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (!imu->initialized)
    {
        return BNO_ERR_NOT_INITIALIZED;
    }

    uint8_t raw[6];

    error_bno err = bno055_read(
        imu,
        BNO_EUL_DATA,
        raw,
        sizeof(raw)
    );

    if (err != BNO_OK)
    {
        return err;
    }

    /*
     * Register order:
     *
     * Heading
     * Roll
     * Pitch
     */
    data->yaw =
        (float)make_int16(raw[0], raw[1])
        / BNO_EUL_SCALE;

    data->roll =
        (float)make_int16(raw[2], raw[3])
        / BNO_EUL_SCALE;

    data->pitch =
        (float)make_int16(raw[4], raw[5])
        / BNO_EUL_SCALE;

    return BNO_OK;
}


/* ===== Quaternion ===== */

error_bno bno055_quaternion(
    bno055_t *imu,
    bno055_vec4_t *data
)
{
    if (imu == NULL || data == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (!imu->initialized)
    {
        return BNO_ERR_NOT_INITIALIZED;
    }

    uint8_t raw[8];

    error_bno err = bno055_read(
        imu,
        BNO_QUA_DATA,
        raw,
        sizeof(raw)
    );

    if (err != BNO_OK)
    {
        return err;
    }

    data->w =
        (float)make_int16(raw[0], raw[1])
        / BNO_QUA_SCALE;

    data->x =
        (float)make_int16(raw[2], raw[3])
        / BNO_QUA_SCALE;

    data->y =
        (float)make_int16(raw[4], raw[5])
        / BNO_QUA_SCALE;

    data->z =
        (float)make_int16(raw[6], raw[7])
        / BNO_QUA_SCALE;

    return BNO_OK;
}


/* ===== Accelerometer ===== */

error_bno bno055_acc(
    bno055_t *imu,
    bno055_vec3_t *data
)
{
    return read_vec3(
        imu,
        BNO_ACC_DATA,
        BNO_ACC_SCALE,
        data
    );
}


/* ===== Gyroscope ===== */

error_bno bno055_gyro(
    bno055_t *imu,
    bno055_vec3_t *data
)
{
    return read_vec3(
        imu,
        BNO_GYR_DATA,
        BNO_GYR_SCALE,
        data
    );
}


/* ===== Magnetometer ===== */

error_bno bno055_mag(
    bno055_t *imu,
    bno055_vec3_t *data
)
{
    return read_vec3(
        imu,
        BNO_MAG_DATA,
        BNO_MAG_SCALE,
        data
    );
}


/* ===== Linear Acceleration ===== */

error_bno bno055_linear_acc(
    bno055_t *imu,
    bno055_vec3_t *data
)
{
    return read_vec3(
        imu,
        BNO_LIA_DATA,
        BNO_ACC_SCALE,
        data
    );
}


/* ===== Gravity ===== */

error_bno bno055_gravity(
    bno055_t *imu,
    bno055_vec3_t *data
)
{
    return read_vec3(
        imu,
        BNO_GRV_DATA,
        BNO_ACC_SCALE,
        data
    );
}


/* ===== Temperature ===== */

error_bno bno055_temperature(
    bno055_t *imu,
    int8_t *temperature
)
{
    if (imu == NULL || temperature == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (!imu->initialized)
    {
        return BNO_ERR_NOT_INITIALIZED;
    }

    uint8_t raw;

    error_bno err = bno055_read(
        imu,
        BNO_TEMP,
        &raw,
        1
    );

    if (err != BNO_OK)
    {
        return err;
    }

    *temperature = (int8_t)raw;

    return BNO_OK;
}


/* ===== Calibration ===== */

error_bno bno055_calibration(
    bno055_t *imu,
    bno055_calib_t *calib
)
{
    if (imu == NULL || calib == NULL)
    {
        return BNO_ERR_NULL_PTR;
    }

    if (!imu->initialized)
    {
        return BNO_ERR_NOT_INITIALIZED;
    }

    uint8_t raw;

    error_bno err = bno055_read(
        imu,
        BNO_CALIB_STAT,
        &raw,
        1
    );

    if (err != BNO_OK)
    {
        return err;
    }

    calib->sys   = (raw >> 6) & 0x03U;
    calib->gyro  = (raw >> 4) & 0x03U;
    calib->accel = (raw >> 2) & 0x03U;
    calib->mag   =  raw       & 0x03U;

    return BNO_OK;
}


/* ===== Error String ===== */

const char *bno055_err_str(error_bno err)
{
    switch (err)
    {
        case BNO_OK:
            return "OK";

        case BNO_ERR_NULL_PTR:
            return "NULL_PTR";

        case BNO_ERR_I2C:
            return "I2C";

        case BNO_ERR_WRONG_CHIP_ID:
            return "WRONG_CHIP_ID";

        case BNO_ERR_BAD_ARG:
            return "BAD_ARG";

        case BNO_ERR_NOT_INITIALIZED:
            return "NOT_INITIALIZED";

        default:
            return "UNKNOWN";
    }
}