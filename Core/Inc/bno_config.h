#ifndef INC_BNO_CONFIG_H_
#define INC_BNO_CONFIG_H_

/*
 * BNO055 project configuration
 */

/* I2C timeout */
#define BNO055_I2C_TIMEOUT_MS       100U

/* BNO055 reset -> boot time */
#define BNO055_RESET_DELAY_MS       700U

/* operation mode change wait */
#define BNO055_MODE_DELAY_MS        25U

/* I2C retry */
#define BNO055_BOOT_RETRY_COUNT     10U
#define BNO055_BOOT_RETRY_DELAY_MS  100U

#endif