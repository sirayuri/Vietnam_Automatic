#include "cdc_protocol.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "bno055.h"
#include "usbd_cdc_if.h"

#define CDC_RX_SIZE 128

extern volatile uint16_t line_adc_right[8];
extern volatile uint16_t line_adc_center[8];
extern volatile bool limit_bottom;
extern volatile bool limit_top;
extern volatile uint16_t TOF_arm;
extern volatile uint16_t TOF_hall;

//USBバッファ関係
static char rx_buf[CDC_RX_SIZE];
static char cmd_buf[CDC_RX_SIZE];
static char tx_buf[CDC_RX_SIZE];

static uint16_t rx_index = 0;
static volatile uint8_t cmd_ready = 0;

static void send_text(const char *text)
{
    (void)CDC_Transmit_FS((uint8_t *)text, (uint16_t)strlen(text));
}

void CDC_Protocol_Task(void)
{
    if (!cmd_ready)
        return;

    CDC_Protocol_Process(cmd_buf);

    cmd_ready = 0;
}

void CDC_Debug_Task(void)
{
    #if DEBUG_IMU

    static uint32_t last_tick = 0;

    uint32_t now = HAL_GetTick();

    if ((now - last_tick) < IMU_DEBUG_PERIOD_MS)
        return;

    last_tick = now;

    bno055_euler_t euler;

    error_bno err = bno055_euler(&bno, &euler);

    if (err != BNO_OK)
        return;

    int len = snprintf(
        tx_buf,
        sizeof(tx_buf),
        "IMU %.2f %.2f %.2f\r\n",
        euler.roll,
        euler.pitch,
        euler.yaw
    );

    if (len > 0)
    {
        (void)CDC_Transmit_FS((uint8_t *)tx_buf, (uint16_t)len);
    }

    #endif

    #if DEBUG_POWER
        static uint32_t last_tick = 0;

        uint32_t now = HAL_GetTick();

        if ((now - last_tick) < IMU_DEBUG_PERIOD_MS)
            return;

        last_tick = now;

        float voltage = Power_GetVoltage();

        if (voltage < 0.0f)
        {
            send_text("ERR POWER ADC\r\n");
            return;
        }

        snprintf(
            tx_buf,
            sizeof(tx_buf),
            "POWER %.2f\r\n",
            voltage
        );

        send_text(tx_buf);
    #endif
}

void CDC_Protocol_Receive(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        if (buf[i] == '\r' || buf[i] == '\n')
        {
            // CRLF の2文字目など、空行は無視
            if (rx_index == 0)
                continue;

            if (!cmd_ready)
            {
                memcpy(cmd_buf, rx_buf, rx_index);
                cmd_buf[rx_index] = '\0';
                cmd_ready = 1;
            }

            rx_index = 0;
        }
        else if (rx_index < sizeof(rx_buf) - 1)
        {
            rx_buf[rx_index++] = buf[i];
        }
    }
}

void CDC_Protocol_LowVoltageNotify(float voltage)
{
    snprintf(
        tx_buf,
        sizeof(tx_buf),
        "FAULT LOW_VOLTAGE %.2f\r\n",
        voltage
    );

    send_text(tx_buf);
}

void CDC_Protocol_Process(char *rx)
{
    unsigned int value1;
    unsigned int value2;
    unsigned int line_values[8];

    if (strcmp(rx, "PING") == 0)
    {
        send_text("PONG\r\n");
    }
    else if (sscanf(rx, "SEND LINE %u %u %u %u %u %u %u %u",
                    &line_values[0], &line_values[1], &line_values[2],
                    &line_values[3], &line_values[4], &line_values[5],
                    &line_values[6], &line_values[7]) == 8)
    {
        for (unsigned int i = 0; i < 8U; i++)
        {
            if (line_values[i] > UINT16_MAX)
            {
                send_text("ERR LINE RANGE\r\n");
                return;
            }
            line_adc_center[i] = (uint16_t)line_values[i];
        }

        snprintf(
            tx_buf, sizeof(tx_buf),
            "OK LINE %u %u %u %u %u %u %u %u\r\n",
            (unsigned int)line_adc_center[0], (unsigned int)line_adc_center[1],
            (unsigned int)line_adc_center[2], (unsigned int)line_adc_center[3],
            (unsigned int)line_adc_center[4], (unsigned int)line_adc_center[5],
            (unsigned int)line_adc_center[6], (unsigned int)line_adc_center[7]
        );
        send_text(tx_buf);
    }
    else if ((strcmp(rx, "SEND LINE") == 0) || (strcmp(rx, "GET LINE") == 0))
    {
        snprintf(
            tx_buf, sizeof(tx_buf),
            "LINE %u %u %u %u %u %u %u %u\r\n",
            (unsigned int)line_adc_center[0], (unsigned int)line_adc_center[1],
            (unsigned int)line_adc_center[2], (unsigned int)line_adc_center[3],
            (unsigned int)line_adc_center[4], (unsigned int)line_adc_center[5],
            (unsigned int)line_adc_center[6], (unsigned int)line_adc_center[7]
        );
        send_text(tx_buf);
    }
    else if (strcmp(rx, "GET LINE RIGHT") == 0)
    {
        snprintf(
            tx_buf, sizeof(tx_buf),
            "LINE_RIGHT %u %u %u %u %u %u %u %u\r\n",
            (unsigned int)line_adc_right
        [0], (unsigned int)line_adc_right
        [1],
            (unsigned int)line_adc_right
        [2], (unsigned int)line_adc_right
        [3],
            (unsigned int)line_adc_right
        [4], (unsigned int)line_adc_right
        [5],
            (unsigned int)line_adc_right
        [6], (unsigned int)line_adc_right
        [7]
        );
        send_text(tx_buf);
    }
    else if (sscanf(rx, "SEND TOF %u %u", &value1, &value2) == 2)
    {
        if ((value1 > UINT16_MAX) || (value2 > UINT16_MAX))
        {
            send_text("ERR TOF RANGE\r\n");
            return;
        }

        TOF_arm = (uint16_t)value1;
        TOF_hall = (uint16_t)value2;
        snprintf(tx_buf, sizeof(tx_buf), "OK TOF %u %u\r\n", (unsigned int)TOF_arm, (unsigned int)TOF_hall);
        send_text(tx_buf);
    }
    else if (strcmp(rx, "GET TOF") == 0)
    {
        snprintf(tx_buf, sizeof(tx_buf), "TOF %u %u\r\n",
                 (unsigned int)TOF_arm, (unsigned int)TOF_hall);
        send_text(tx_buf);
    }
    else if ((sscanf(rx, "SEND LIMIT %u %u", &value1, &value2) == 2))
    {
        if ((value1 > 1U) || (value2 > 1U))
        {
            send_text("ERR LIMIT VALUE\r\n");
            return;
        }

        /* Limit switches are measured by the dedicated sensor MCU. */
        limit_bottom = (value1 != 0U);
        limit_top = (value2 != 0U);
        snprintf(tx_buf, sizeof(tx_buf), "OK LIMIT %u %u\r\n", limit_bottom ? 1U : 0U, limit_top ? 1U : 0U);
        send_text(tx_buf);
    }
    else if (strcmp(rx, "GET LIMIT") == 0)
    {
        snprintf(tx_buf, sizeof(tx_buf), "LIMIT %u %u\r\n",
                 limit_bottom ? 1U : 0U, limit_top ? 1U : 0U);
        send_text(tx_buf);
    }
    // else if (strcmp(rx, "GET POWER") == 0)
    // {
    // #if USE_POWER_OBSERVE

    //     float voltage = Power_GetVoltage();

    //     if (voltage < 0.0f)
    //     {
    //         send_text("ERR POWER ADC\r\n");
    //         return;
    //     }

    //     snprintf(
    //         tx_buf,
    //         sizeof(tx_buf),
    //         "POWER %.2f\r\n",
    //         voltage
    //     );

    //     send_text(tx_buf);
    // #else

    //     send_text("ERR POWER DISABLED\r\n");

    // #endif
    // }
    // else if (strcmp(rx, "GET LINE") == 0)
    // {
    //     #if USE_LINE_TRACE

    //         snprintf(
    //             tx_buf,
    //             sizeof(tx_buf),
    //             "LINE %u %u %u %u %u %u %u %u\r\n",
    //             line_adc[0],
    //             line_adc[1],
    //             line_adc[2],
    //             line_adc[3],
    //             line_adc[4],
    //             line_adc[5],
    //             line_adc[6],
    //             line_adc[7]
    //         );

    //         send_text(tx_buf);

    //     #else

    //         send_text("ERR LINE_SENSOR DISABLED\r\n");

    //     #endif
    // }
    // else if (strcmp(rx, "GET IMU") == 0){
    //     #if USE_BNO055
    //         bno055_euler_t euler;

    //         error_bno err = bno055_euler(
    //             &bno,
    //             &euler
    //         );

    //         if (err == BNO_OK)
    //         {
    //             snprintf(
    //                 tx_buf,
    //                 sizeof(tx_buf),
    //                 "IMU %.2f %.2f %.2f\r\n",
    //                 euler.roll,
    //                 euler.pitch,
    //                 euler.yaw
    //             );

    //             send_text(tx_buf);
    //         }
    //         else
    //         {
    //             snprintf(
    //                 tx_buf,
    //                 sizeof(tx_buf),
    //                 "ERR IMU %s\r\n",
    //                 bno055_err_str(err)
    //             );

    //             send_text(tx_buf);
    //         }
    //     #else
    //         send_text("ERR BNO055 DISABLED\r\n");
    //         return;
    //     #endif
    // }
    // else if (strcmp(rx, "GET LIMIT") == 0) {
    //     #if USE_LIMIT_SW
    //         snprintf(tx_buf, sizeof(tx_buf), "LIMIT %d %d\r\n", HAL_GPIO_ReadPin(Limit_SW_1_GPIO_Port, Limit_SW_1_Pin), HAL_GPIO_ReadPin(Limit_SW_2_GPIO_Port, Limit_SW_2_Pin));
    //         send_text(tx_buf);
    //     #else
    //         send_text("ERR LIMIT_SW DISABLED\r\n");
    //         return;
    //     #endif
    // }
    // else if (strncmp(rx, "MOTOR ", 6) == 0)
    // {
    //     int id;
    //     int power;

    //     if (sscanf(rx, "MOTOR %d %d", &id, &power) != 2)
    //     {
    //         send_text("ERR MOTOR FORMAT\r\n");
    //         return;
    //     }

    //     /* power limit */
    //     if (power > MOTOR_PWM_LIMIT)
    //         power = MOTOR_PWM_LIMIT;

    //     if (power < -MOTOR_PWM_LIMIT)
    //         power = -MOTOR_PWM_LIMIT;


    //     TIM_HandleTypeDef *htim = NULL;
    //     uint32_t ch_a = 0;
    //     uint32_t ch_b = 0;

    //     switch (id)
    //     {
    //         case 1:
    //     #if MOTOR_1_ENABLE
    //             htim = &htim1;
    //             ch_a = TIM_CHANNEL_2;
    //             ch_b = TIM_CHANNEL_1;
    //     #else
    //             send_text("ERR MOTOR DISABLED\r\n");
    //             return;
    //     #endif
    //             break;

    //         case 2:
    //     #if MOTOR_2_ENABLE
    //             htim = &htim3;
    //             ch_a = TIM_CHANNEL_4;
    //             ch_b = TIM_CHANNEL_3;
    //     #else
    //             send_text("ERR MOTOR DISABLED\r\n");
    //             return;
    //     #endif
    //             break;

    //         case 3:
    //     #if MOTOR_3_ENABLE
    //             htim = &htim3;
    //             ch_a = TIM_CHANNEL_2;
    //             ch_b = TIM_CHANNEL_1;
    //     #else
    //             send_text("ERR MOTOR DISABLED\r\n");
    //             return;
    //     #endif
    //             break;

    //         case 4:
    //     #if MOTOR_4_ENABLE
    //             htim = &htim2;
    //             ch_a = TIM_CHANNEL_4;
    //             ch_b = TIM_CHANNEL_3;
    //     #else
    //             send_text("ERR MOTOR DISABLED\r\n");
    //             return;
    //     #endif
    //             break;

    //         case 5:
    //     #if MOTOR_5_ENABLE
    //             htim = &htim2;
    //             ch_a = TIM_CHANNEL_1;
    //             ch_b = TIM_CHANNEL_2;
    //     #else
    //             send_text("ERR MOTOR DISABLED\r\n");
    //             return;
    //     #endif
    //             break;

    //         case 6:
    //     #if MOTOR_6_ENABLE
    //             htim = &htim4;
    //             ch_a = TIM_CHANNEL_1;
    //             ch_b = TIM_CHANNEL_2;
    //     #else
    //             send_text("ERR MOTOR DISABLED\r\n");
    //             return;
    //     #endif
    //             break;

    //         default:
    //             send_text("ERR MOTOR ID\r\n");
    //             return;
    //     }

    //     #if ENABLE_LOW_VOLTAGE_PROTECTION
    //         if (low_voltage_fault)
    //         {
    //             __HAL_TIM_SET_COMPARE(htim, ch_a, 0);
    //             __HAL_TIM_SET_COMPARE(htim, ch_b, 0);

    //             send_text("ERR LOW VOLTAGE\r\n");
    //             return;
    //         }
    //     #endif

    //     #if ENABLE_MOTOR_TIMEOUT
    //         last_motor_command_tick = HAL_GetTick();
    //         motor_timeout_fault = 0;
    //     #endif

    //     if (power > 0)
    //     {
    //         __HAL_TIM_SET_COMPARE(htim, ch_a, power);
    //         __HAL_TIM_SET_COMPARE(htim, ch_b, 0);
    //     }
    //     else if (power < 0)
    //     {
    //         __HAL_TIM_SET_COMPARE(htim, ch_a, 0);
    //         __HAL_TIM_SET_COMPARE(htim, ch_b, -power);
    //     }
    //     else
    //     {
    //         /* coast */
    //         __HAL_TIM_SET_COMPARE(htim, ch_a, 0);
    //         __HAL_TIM_SET_COMPARE(htim, ch_b, 0);
    //     }


    //     snprintf(
    //         tx_buf,
    //         sizeof(tx_buf),
    //         "OK MOTOR %d %d\r\n",
    //         id,
    //         power
    //     );

    //     send_text(tx_buf);
    // }
    else
    {
        send_text("ERR UNKNOWN_COMMAND\r\n");
    }
}
