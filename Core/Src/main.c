/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdbool.h"
#include "cdc_protocol.h"
#include "bno055.h"
#include "bno_config.h"
#include <stdint.h>
#include <math.h>
#include <stddef.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_PWM_LIMIT          800.0f
#define ARM_VERTICAL_PWM_LIMIT   900.0f
#define MOVE_CONTROL_PERIOD_MS   10U
#define PID_INTEGRAL_LIMIT        1000.0f
#define DEG_TO_RAD                0.01745329251994329577f

#define LINE_SENSOR_COUNT         8U
#define POINT_CONFIRM_MS          300U
#define FRONT_LINE_CONFIRM_MS     300U
#define POINT_REARM_CLEAR_MS      120U
#define LINE_CENTER_STALE_MS      200U
#define FRONT_LINE_MIN_BLACK      3U
#define POINT_RIGHT_MIN_BLACK     3U
#define POINT_SEARCH_POWER        700.0f
#define P5_POINT_SEARCH_POWER     600.0f
#define P9_POINT_SEARCH_POWER     600.0f
#define POINT_ALIGN_POWER         700.0f
#define POINT_MIN_START_PWM       650.0f
#define DONUT_MOVE_POWER          700.0f
#define DONUT_TOF_STALE_MS        600U
#define DONUT_TOF_CONFIRM_MS      150U
#define DONUT_TOF_TOLERANCE_MM    15U
#define DONUT_MIN_DISTANCE_DROP_MM 30U
#define DONUT_MOVE_TIMEOUT_MS     10000U
#define DONUT_GUIDE_APPROACH_POWER 650.0f
#define DONUT_GUIDE_APPROACH_MS    1200U
#define ARM_LOWER_POWER            900.0f
#define ARM_STARTUP_HOME_POWER     850.0f
#define ARM_GRIPPER_CLOSE_POWER    700.0f
#define ARM_GRIPPER_HOLD_TOF_MM     75U
#define ARM_GRIPPER_CONFIRM_MS      95U
#define ARM_GRIPPER_HYSTERESIS_MM     5U
#define ARM_GRIPPER_TOF_STALE_MS     600U
#define ARM_GRIPPER_CLOSE_TIMEOUT_MS 5000U
#define ARM_GRIPPER_OPEN_POWER       700.0f
#define ARM_GRIPPER_OPEN_TOF_MM      95U
#define ARM_GRIPPER_OPEN_CONFIRM_MS  100U
#define ARM_GRIPPER_OPEN_TIMEOUT_MS 5000U
#define ARM_HALF_RAISE_MS          3000U
#define ROUTE_FINAL_FORWARD_POINTS    3U
#define ROUTE_P5_TO_P7_POINTS          2U
#define LINE_TRACE_POWER            700.0f
#define LINE_TRACE_MAX_CORRECTION     0.10f
#define LINE_TRACE_CONTROL_PERIOD_MS  10U
#define BODY_TURN_MIN_PWM            600.0f
#define BODY_TURN_MAX_PWM            750.0f
#define BODY_TURN_TOLERANCE_DEG        5.0f
#define BODY_TURN_STABLE_MS          250U
#define BODY_TURN_TIMEOUT_MS        8000U
#define BRIDGE_IMU_SAMPLE_MS          20U
#define BRIDGE_SLOPE_ENTER_DEG         8.0f
#define BRIDGE_FLAT_RETURN_DEG          4.0f
#define BRIDGE_SLOPE_CONFIRM_MS       250U
#define BRIDGE_FLAT_CONFIRM_MS        400U
#define BRIDGE_TOP_FORWARD_MS        200U
#define BRIDGE_STOP_BEFORE_TURN_MS    300U
#define BRIDGE_STAGE_TIMEOUT_MS     20000U
#define P8_ARM_RAISE_TIMEOUT_MS     10000U
#define P8_LIMIT_STALE_MS            1000U
#define P8_FORWARD_TRACE_MS          3000U
#define P8_PREOPEN_REVERSE_MS         100U
#define P8_PREOPEN_REVERSE_POWER    700.0f
#define P9_DIAGONAL_COMPONENT          0.5f
#define P9_ALIGN_TIMEOUT_MS           3000U
#define P9_EARLY_LINE_IGNORE_MS       5000U
#define P9_FRONT_LINE_CONFIRM_MS       600U
#define P9_FRONT_LINE_MIN_CONFIDENCE 1500.0f

/* Set to 0 to restore the legacy 3.5 / 3.5 sensor-center targets. */
#define USE_MEASURED_LINE_CENTERS        0U
#if USE_MEASURED_LINE_CENTERS
#define FRONT_LINE_CENTER_POSITION      4.0f
#define RIGHT_LINE_CENTER_POSITION      3.4f
#else
#define FRONT_LINE_CENTER_POSITION      3.5f
#define RIGHT_LINE_CENTER_POSITION      3.5f
#endif

#define LINE_RED_SIGMA_FLOOR       20.0f
#define ENABLE_ARM_VERTICAL       1U
#define POINT_SEARCH_TIMEOUT_MS   10000U
#define POINT_ALIGN_TIMEOUT_MS    3000U
#define ROUTE_ALIGN_TIMEOUT_MS    1000U
#define FRONT_LINE_ALIGN_TIMEOUT_MS 3000U
#define FRONT_LINE_FOUND_STABLE_MS  60U
#define POINT_ALIGN_STABLE_MS     100U
#define POINT_ALIGN_ERROR_X       0.50f
#define POINT_ALIGN_ERROR_Y       0.50f
#define POINT_LED_ON_LEVEL        GPIO_PIN_SET
#define POINT_LED_OFF_LEVEL       GPIO_PIN_RESET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

void move_degree(float body_degree, float vx, float vy, float drive_power);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* The LINE sensor connected to this MCU is the right sensor (8 ADC channels). */
volatile uint16_t line_adc_right[8];
/* Front LINE values received from the dedicated sensor MCU via Raspberry Pi. */
volatile uint16_t line_adc_center[8];

/* Set when a new front-line packet is accepted by CDC_Protocol_Process(). */
volatile uint32_t line_adc_center_last_update_tick = 0U;

/* Point detection status.  point_event is a one-loop pulse. */
volatile bool point_detected = false;
volatile bool point_event = false;
volatile bool point_arrived = false;
volatile bool auto_control_error = false;
volatile uint32_t current_point_index = 0U;
volatile bool point_detection_enabled = false;

volatile bool limit_bottom = false;
volatile bool limit_top = false;
volatile uint32_t limit_last_update_tick = 0U;

volatile uint16_t TOF_arm = 0;
volatile uint16_t TOF_hall = 0;
volatile uint32_t tof_last_update_tick = 0U;

/* Configure these later after measuring the actual TOF values. */
volatile uint16_t donut_floor_distance_mm = 430U;
volatile uint16_t donut_surface_distance_mm = 380U;
volatile uint16_t donut_tof_tolerance_mm = DONUT_TOF_TOLERANCE_MM;

/* Physical TOF roles and CDC order: TOF <HALL> <ARM>.
 *   TOF_hall = arm height / downward floor-donut distance
 *   TOF_arm  = gripper opening/holding distance
 */
/* 0 = TOF_arm, 1 = TOF_hall. */
volatile uint8_t donut_tof_source = 1;

/* Tune this time to match the distance from the TOF trigger point to the arm guide. */
volatile uint32_t donut_guide_approach_time_ms = DONUT_GUIDE_APPROACH_MS;

/* +PWM = arm down, -PWM = arm up.  Kept at zero until the arm is tested. */
volatile float arm_vertical_command = 0.0f;
/* Motor 5: +PWM opens the gripper, -PWM closes it. */
volatile float arm_gripper_command = 0.0f;

float P_gain = 9.0;
float I_gain = 0.0;
float D_gain = 0.0;
float point_align_P_gain_x = 0.35f;
float point_align_P_gain_y = 0.35f;
float point_align_I_gain_x = 0.0f;
float point_align_I_gain_y = 0.0f;
float point_align_D_gain_x = 0.0f;
float point_align_D_gain_y = 0.0f;
/* Front-line PID output is robot-relative vx while vy is fixed forward/back. */
float line_trace_P_gain = 0.06f;
float line_trace_I_gain = 0.0f;
float line_trace_D_gain = 0.0f;
/* Right-line PID output is robot-relative vy while vx is fixed left/right. */
float right_line_trace_P_gain = 0.06f;
float right_line_trace_I_gain = 0.0f;
float right_line_trace_D_gain = 0.0f;
float body_turn_P_gain = 20.0f;
float PID_err = 0.0;
float PID_err_old = 0.0;
float degree_old = 0.0;

/* Set these values from the automatic-control code.  Zero speed is safe. */
volatile float target_body_degree = 0.0f;
volatile float target_move_degree = 0.0f;

static float normalize_degree(float degree)
{
  while (degree > 180.0f)
  {
    degree -= 360.0f;
  }
  while (degree < -180.0f)
  {
    degree += 360.0f;
  }
  return degree;
}

static float limit_float(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

typedef struct
{
  uint8_t mask;
  uint8_t black_count;
  uint8_t longest_run;
  float score;
} line_sensor_features_t;

typedef struct
{
  float position;
  float confidence;
  uint8_t black_count;
  bool valid;
} line_sensor_position_t;

/*
 * These values are initial background models obtained from the supplied
 * white/green floor logs.  The threshold is the larger of the white and
 * green background limits, plus a margin.  The front sensor also uses a
 * recalculated red-floor background model.  Black is expected to produce a
 * larger ADC value.
 */
static const float line_right_white_mean[LINE_SENSOR_COUNT] =
  {3025.0f, 3232.0f, 2745.0f, 2715.0f, 2860.0f, 3133.0f, 2900.0f, 3098.0f};
static const float line_right_white_sigma[LINE_SENSOR_COUNT] =
  {  20.0f,   12.0f,   30.0f,   30.0f,   30.0f,   12.0f,   18.0f,   20.0f};
static const float line_right_green_mean[LINE_SENSOR_COUNT] =
  {3248.0f, 3177.0f, 2561.0f, 2483.0f, 2776.0f, 3063.0f, 2843.0f, 3071.0f};
static const float line_right_green_sigma[LINE_SENSOR_COUNT] =
  {   5.0f,    6.0f,    6.0f,    6.0f,    6.0f,    5.0f,    6.0f,    8.0f};
/* Red-floor model is used for right-sensor position calculation only. */
static const float line_right_red_mean[LINE_SENSOR_COUNT] =
  {3247.46f, 3196.08f, 2615.85f, 2514.88f, 2793.15f, 3094.15f, 2889.23f, 3108.58f};
static const float line_right_red_sigma[LINE_SENSOR_COUNT] =
  {  27.41f,   10.12f,   21.18f,    7.86f,    8.73f,   21.19f,   20.96f,    9.94f};
static const float line_front_white_mean[LINE_SENSOR_COUNT] =
  {1910.0f, 1540.0f, 1835.0f, 1055.0f, 1410.0f, 1320.0f, 740.0f, 970.0f};
static const float line_front_white_sigma[LINE_SENSOR_COUNT] =
  {  12.0f,   12.0f,   10.0f,    8.0f,   10.0f,   10.0f,  12.0f,  10.0f};
static const float line_front_green_mean[LINE_SENSOR_COUNT] =
  {1787.0f, 1628.0f, 1868.0f,  645.0f,  731.0f, 1215.0f, 387.0f, 610.0f};
static const float line_front_green_sigma[LINE_SENSOR_COUNT] =
  {   8.0f,    7.0f,    7.0f,    7.0f,    7.0f,    7.0f,   7.0f,   8.0f};
/* Recalculated from the latest 29 red-floor samples supplied by the user. */
static const float line_front_red_mean[LINE_SENSOR_COUNT] =
  {1994.79f, 1856.55f, 1866.79f, 1153.62f, 1248.93f, 1304.93f, 558.10f, 929.76f};
static const float line_front_red_sigma[LINE_SENSOR_COUNT] =
  {  46.47f,    8.88f,    9.03f, 133.20f,   14.66f,   52.80f,  26.85f, 112.37f};

typedef enum
{
  POINT_STATE_ARMED = 0,
  POINT_STATE_CANDIDATE,
  POINT_STATE_LATCHED,
  POINT_STATE_WAIT_REARM
} point_state_t;

static point_state_t point_state = POINT_STATE_ARMED;
static uint32_t point_candidate_tick = 0U;
static uint32_t point_clear_tick = 0U;

static float background_limit(float mean, float sigma)
{
  /* Four sigma rejects the measured floor noise; the margin covers drift. */
  return mean + (4.0f * sigma) + 120.0f;
}

static float red_background_limit(float mean, float sigma)
{
  /* Static red-floor logs are very stable; reserve margin for motion. */
  if (sigma < LINE_RED_SIGMA_FLOOR)
  {
    sigma = LINE_RED_SIGMA_FLOOR;
  }
  return background_limit(mean, sigma);
}

static void get_line_sensor_features(
    const volatile uint16_t *raw,
    const float *white_mean,
    const float *white_sigma,
    const float *green_mean,
    const float *green_sigma,
    const float *red_mean,
    const float *red_sigma,
    line_sensor_features_t *features)
{
  features->mask = 0U;
  features->black_count = 0U;
  features->longest_run = 0U;
  features->score = 0.0f;

  uint8_t current_run = 0U;

  for (uint32_t i = 0U; i < LINE_SENSOR_COUNT; i++)
  {
    float white_limit = background_limit(white_mean[i], white_sigma[i]);
    float green_limit = background_limit(green_mean[i], green_sigma[i]);
    float threshold = (white_limit > green_limit) ? white_limit : green_limit;

    float white_z = ((float)raw[i] - white_mean[i]) /
                    ((white_sigma[i] > 1.0f) ? white_sigma[i] : 1.0f);
    float green_z = ((float)raw[i] - green_mean[i]) /
                    ((green_sigma[i] > 1.0f) ? green_sigma[i] : 1.0f);
    float z = (white_z > green_z) ? white_z : green_z;

    if ((red_mean != NULL) && (red_sigma != NULL))
    {
      float red_limit = red_background_limit(red_mean[i], red_sigma[i]);
      if (red_limit > threshold)
      {
        threshold = red_limit;
      }

      float red_z = ((float)raw[i] - red_mean[i]) /
                    ((red_sigma[i] > 1.0f) ? red_sigma[i] : 1.0f);
      if (red_z > z)
      {
        z = red_z;
      }
    }

    if ((float)raw[i] > threshold)
    {
      features->mask |= (uint8_t)(1U << i);
      features->black_count++;
      current_run++;
      if (current_run > features->longest_run)
      {
        features->longest_run = current_run;
      }
      features->score += z;
    }
    else
    {
      current_run = 0U;
    }
  }
}

/*
 * A red starting-floor patch can make only front S1 look black.  Do not use
 * line position alone for the initial front-line detection: require a
 * continuous group of at least FRONT_LINE_MIN_BLACK sensors.  The measured
 * vertical guide line can activate all eight sensors, so the active-sensor
 * count must not be used as an upper-limit direction classifier here.
 */
static bool front_line_features_is_present(const line_sensor_features_t *front)
{
  return (front->black_count >= FRONT_LINE_MIN_BLACK) &&
         (front->longest_run >= FRONT_LINE_MIN_BLACK);
}

static bool right_line_features_is_present(const line_sensor_features_t *right)
{
  return (right->black_count >= POINT_RIGHT_MIN_BLACK) &&
         (right->longest_run >= POINT_RIGHT_MIN_BLACK);
}

static void get_line_sensor_position(
    const volatile uint16_t *raw,
    const float *white_mean,
    const float *white_sigma,
    const float *green_mean,
    const float *green_sigma,
    const float *red_mean,
    const float *red_sigma,
    line_sensor_position_t *result)
{
  float weighted_sum = 0.0f;
  float signal_sum = 0.0f;
  uint8_t black_count = 0U;

  for (uint32_t i = 0U; i < LINE_SENSOR_COUNT; i++)
  {
    float floor_reference =
        (white_mean[i] > green_mean[i]) ? white_mean[i] : green_mean[i];
    float sigma =
        (white_sigma[i] > green_sigma[i]) ? white_sigma[i] : green_sigma[i];

    if ((red_mean != NULL) && (red_mean[i] > floor_reference))
    {
      floor_reference = red_mean[i];
    }
    if (red_sigma != NULL)
    {
      float red_sigma_effective =
          (red_sigma[i] > LINE_RED_SIGMA_FLOOR) ?
          red_sigma[i] : LINE_RED_SIGMA_FLOOR;
      if (red_sigma_effective > sigma)
      {
        sigma = red_sigma_effective;
      }
    }
    float noise_gate = (3.0f * sigma) + 40.0f;
    float signal = (float)raw[i] - floor_reference - noise_gate;

    if (signal > 0.0f)
    {
      weighted_sum += signal * (float)i;
      signal_sum += signal;
      black_count++;
    }
  }

  result->position = 3.5f;
  result->confidence = signal_sum;
  result->black_count = black_count;
  result->valid = (signal_sum >= 100.0f);

  if (signal_sum > 0.0f)
  {
    result->position = weighted_sum / signal_sum;
  }
}

static bool point_candidate_is_present(void)
{
  line_sensor_features_t front;
  line_sensor_features_t right;

  uint32_t now = HAL_GetTick();

  if ((line_adc_center_last_update_tick == 0U) ||
      ((now - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
  {
    return false;
  }

  get_line_sensor_features(
      line_adc_center,
      line_front_white_mean,
      line_front_white_sigma,
      line_front_green_mean,
      line_front_green_sigma,
      line_front_red_mean,
      line_front_red_sigma,
      &front);

  get_line_sensor_features(
      line_adc_right,
      line_right_white_mean,
      line_right_white_sigma,
      line_right_green_mean,
      line_right_green_sigma,
      NULL,
      NULL,
      &right);

  /*
   * The front sensor must see the normal guide line.  The right sensor must
   * see a second, continuous black region.  With the current mounting this
   * is the first-pass signature of a transverse line/intersection.
   */
  bool front_line = front_line_features_is_present(&front);
  bool right_cross_line = right_line_features_is_present(&right);

  return front_line && right_cross_line;
}

void PointDetect_Reset(void)
{
  point_state = POINT_STATE_ARMED;
  point_candidate_tick = 0U;
  point_clear_tick = 0U;
  point_detected = false;
  point_event = false;
}

void PointDetect_Task(void)
{
  uint32_t now = HAL_GetTick();

  if (!point_detection_enabled)
  {
    PointDetect_Reset();
    return;
  }

  bool candidate = point_candidate_is_present();

  /* A pulse lets the route controller advance exactly once per point. */
  point_event = false;

  switch (point_state)
  {
    case POINT_STATE_ARMED:
      if (candidate)
      {
        point_candidate_tick = now;
        point_state = POINT_STATE_CANDIDATE;
      }
      break;

    case POINT_STATE_CANDIDATE:
      if (!candidate)
      {
        point_state = POINT_STATE_ARMED;
      }
      else if ((now - point_candidate_tick) >= POINT_CONFIRM_MS)
      {
        point_detected = true;
        point_event = true;
        point_clear_tick = now;
        point_state = POINT_STATE_LATCHED;
      }
      break;

    case POINT_STATE_LATCHED:
      /* Keep the event latched while the robot changes to its point flow. */
      if (!candidate)
      {
        point_clear_tick = now;
        point_state = POINT_STATE_WAIT_REARM;
      }
      break;

    case POINT_STATE_WAIT_REARM:
      if (candidate)
      {
        point_state = POINT_STATE_LATCHED;
      }
      else if ((now - point_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        point_detected = false;
        point_state = POINT_STATE_ARMED;
      }
      break;

    default:
      PointDetect_Reset();
      break;
  }
}

/* Stop as soon as an intersection candidate appears.  Confirmation then
 * happens while stationary, preventing the 300 ms confirmation interval from
 * carrying the chassis past the center of the intersection. */
static void point_search_move_with_power(
    float body_degree,
    float vx,
    float vy,
    float drive_power)
{
  if (point_state == POINT_STATE_CANDIDATE)
  {
    move_degree(body_degree, 0.0f, 0.0f, 0.0f);
    return;
  }

  move_degree(body_degree, vx, vy, drive_power);
}

static void point_search_move(float body_degree, float vx, float vy)
{
  point_search_move_with_power(
      body_degree,
      vx,
      vy,
      POINT_SEARCH_POWER);
}

static void set_motor_pwm_with_limit(TIM_HandleTypeDef *timer,
                                     uint32_t forward_channel,
                                     uint32_t reverse_channel,
                                     float command,
                                     float pwm_limit)
{
  command = limit_float(command, -pwm_limit, pwm_limit);

  if (command >= 0.0f)
  {
    __HAL_TIM_SET_COMPARE(timer, forward_channel, (uint32_t)command);
    __HAL_TIM_SET_COMPARE(timer, reverse_channel, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(timer, forward_channel, 0U);
    __HAL_TIM_SET_COMPARE(timer, reverse_channel, (uint32_t)(-command));
  }
}

static void set_motor_pwm(TIM_HandleTypeDef *timer,
                          uint32_t forward_channel,
                          uint32_t reverse_channel,
                          float command)
{
  set_motor_pwm_with_limit(
      timer,
      forward_channel,
      reverse_channel,
      command,
      MOTOR_PWM_LIMIT);
}

/*
 * Motor 6 vertical arm control.
 *   command > 0 : down  (TIM4 CH1)
 *   command < 0 : up    (TIM4 CH2)
 *
 * limit_top and limit_bottom are the active-state booleans received from the
 * dedicated sensor MCU via CDC.  The first LIMIT packet must arrive before
 * the motor is allowed to move; after that, the latest received state is used.
 */
static void ArmVertical_Move(float command)
{
#if ENABLE_ARM_VERTICAL
  if (limit_last_update_tick == 0U)
  {
    set_motor_pwm_with_limit(
        &htim4,
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        0.0f,
        ARM_VERTICAL_PWM_LIMIT);
    return;
  }

  /* Positive command is down, so the bottom switch blocks it. */
  if ((command > 0.0f) && limit_bottom)
  {
    command = 0.0f;
  }

  /* Negative command is up, so the top switch blocks it. */
  if ((command < 0.0f) && limit_top)
  {
    command = 0.0f;
  }

  set_motor_pwm_with_limit(
      &htim4,
      TIM_CHANNEL_1,
      TIM_CHANNEL_2,
      command,
      ARM_VERTICAL_PWM_LIMIT);
#else
  /* Keep motor 6 stopped while arm control is disabled for testing. */
  (void)command;
  set_motor_pwm_with_limit(
      &htim4,
      TIM_CHANNEL_1,
      TIM_CHANNEL_2,
      0.0f,
      ARM_VERTICAL_PWM_LIMIT);
#endif
}

/* Motor 5 gripper control.
 *   command > 0 : open  (TIM2 CH1)
 *   command < 0 : close (TIM2 CH2)
 */
static void ArmGripper_Move(float command)
{
  set_motor_pwm(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, command);
}

/* Startup homing state for motor 6.  The robot must not start its drive flow
 * until the arm has reached the top limit once after power-up. */
static bool arm_startup_home_done = false;

static bool ArmStartupHome_Task(void)
{
  /* Keep the base stopped while the arm is homing. */
  move_degree(0.0f, 0.0f, 0.0f, 0.0f);
  point_detection_enabled = false;

  if (arm_startup_home_done)
  {
    arm_vertical_command = 0.0f;
    return arm_startup_home_done;
  }

  /* Wait indefinitely for the Raspberry Pi/sensor MCU to start sending
   * LIMIT packets.  Do not start the arm before the first packet arrives. */
  if (limit_last_update_tick == 0U)
  {
    arm_vertical_command = 0.0f;
    return false;
  }

  /* Already at the top: homing is complete without moving the arm. */
  if (limit_top)
  {
    arm_vertical_command = 0.0f;
    arm_startup_home_done = true;
    return true;
  }

  /* Motor 6: negative command is up. */
  arm_vertical_command = -ARM_STARTUP_HOME_POWER;
  return false;
}

static void stop_drive_motors(void)
{
  set_motor_pwm(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, 0.0f); /* M1 RR */
  set_motor_pwm(&htim3, TIM_CHANNEL_4, TIM_CHANNEL_3, 0.0f); /* M2 RL */
  set_motor_pwm(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, 0.0f); /* M3 FL */
  set_motor_pwm(&htim2, TIM_CHANNEL_4, TIM_CHANNEL_3, 0.0f); /* M4 FR */
}

static void start_motor_pwm(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* The front sensor is perpendicular to the travel line.  Its PID output is a
 * small mecanum vx correction; keeping it below 0.10 leaves every wheel near
 * the 600-PWM region needed by the motor drivers while vy is +/-1. */
static float line_trace_integral = 0.0f;
static float line_trace_previous_error = 0.0f;
static float line_trace_correction = 0.0f;
static uint32_t line_trace_last_sample_tick = 0U;

static void line_trace_reset(void)
{
  line_trace_integral = 0.0f;
  line_trace_previous_error = 0.0f;
  line_trace_correction = 0.0f;
  line_trace_last_sample_tick = 0U;
}

static bool line_trace_move(float body_degree, float vy)
{
  uint32_t now = HAL_GetTick();
  uint32_t sample_tick = line_adc_center_last_update_tick;
  line_sensor_position_t front;

  if ((sample_tick == 0U) ||
      ((now - sample_tick) > LINE_CENTER_STALE_MS))
  {
    line_trace_reset();
    move_degree(body_degree, 0.0f, 0.0f, 0.0f);
    return false;
  }

  get_line_sensor_position(
      line_adc_center,
      line_front_white_mean,
      line_front_white_sigma,
      line_front_green_mean,
      line_front_green_sigma,
      line_front_red_mean,
      line_front_red_sigma,
      &front);

  if (!front.valid)
  {
    line_trace_reset();
    move_degree(body_degree, 0.0f, 0.0f, 0.0f);
    return false;
  }

  /* Update PID once per newly received sensor packet. */
  if (sample_tick != line_trace_last_sample_tick)
  {
    float error = FRONT_LINE_CENTER_POSITION - front.position;
    float dt = (line_trace_last_sample_tick == 0U) ?
               ((float)LINE_TRACE_CONTROL_PERIOD_MS / 1000.0f) :
               ((float)(sample_tick - line_trace_last_sample_tick) / 1000.0f);
    if (dt < 0.001f)
    {
      dt = 0.001f;
    }

    line_trace_integral += error * dt;
    line_trace_integral = limit_float(line_trace_integral, -3.0f, 3.0f);

    float derivative = (line_trace_last_sample_tick == 0U) ?
                       0.0f :
                       ((error - line_trace_previous_error) / dt);

    line_trace_correction =
        (line_trace_P_gain * error) +
        (line_trace_I_gain * line_trace_integral) +
        (line_trace_D_gain * derivative);
    line_trace_correction = limit_float(
        line_trace_correction,
        -LINE_TRACE_MAX_CORRECTION,
        LINE_TRACE_MAX_CORRECTION);

    line_trace_previous_error = error;
    line_trace_last_sample_tick = sample_tick;
  }

  move_degree(
      body_degree,
      line_trace_correction,
      (vy >= 0.0f) ? 1.0f : -1.0f,
      LINE_TRACE_POWER);
  return true;
}

/* Stop on the first intersection candidate so the existing 300-ms point
 * confirmation can finish without the chassis crossing the whole point. */
static bool line_trace_point_search(float body_degree, float vy)
{
  if (point_state == POINT_STATE_CANDIDATE)
  {
    line_trace_reset();
    move_degree(body_degree, 0.0f, 0.0f, 0.0f);
    return true;
  }

  return line_trace_move(body_degree, vy);
}

/* During a right/left traverse, the right sensor crosses the guide line in
 * the robot's forward/rear direction.  Keep vx fixed and use line-position
 * PID as a small vy correction.  move_degree() simultaneously holds yaw. */
static float right_line_trace_integral = 0.0f;
static float right_line_trace_previous_error = 0.0f;
static float right_line_trace_correction = 0.0f;
static uint32_t right_line_trace_last_sample_tick = 0U;

static void right_line_trace_reset(void)
{
  right_line_trace_integral = 0.0f;
  right_line_trace_previous_error = 0.0f;
  right_line_trace_correction = 0.0f;
  right_line_trace_last_sample_tick = 0U;
}

static bool right_line_trace_move(
    float body_degree,
    float vx,
    float drive_power)
{
  uint32_t now = HAL_GetTick();
  line_sensor_position_t right;

  get_line_sensor_position(
      line_adc_right,
      line_right_white_mean,
      line_right_white_sigma,
      line_right_green_mean,
      line_right_green_sigma,
      line_right_red_mean,
      line_right_red_sigma,
      &right);

  if (!right.valid)
  {
    right_line_trace_reset();
    move_degree(body_degree, 0.0f, 0.0f, 0.0f);
    return false;
  }

  if ((right_line_trace_last_sample_tick == 0U) ||
      ((now - right_line_trace_last_sample_tick) >=
       LINE_TRACE_CONTROL_PERIOD_MS))
  {
    /* Right S0 is rear and S7 is front: positive error moves forward. */
    float error = right.position - RIGHT_LINE_CENTER_POSITION;
    float dt = (right_line_trace_last_sample_tick == 0U) ?
               ((float)LINE_TRACE_CONTROL_PERIOD_MS / 1000.0f) :
               ((float)(now - right_line_trace_last_sample_tick) / 1000.0f);
    if (dt < 0.001f)
    {
      dt = 0.001f;
    }

    right_line_trace_integral += error * dt;
    right_line_trace_integral = limit_float(
        right_line_trace_integral,
        -3.0f,
        3.0f);

    float derivative = (right_line_trace_last_sample_tick == 0U) ?
                       0.0f :
                       ((error - right_line_trace_previous_error) / dt);

    right_line_trace_correction =
        (right_line_trace_P_gain * error) +
        (right_line_trace_I_gain * right_line_trace_integral) +
        (right_line_trace_D_gain * derivative);
    right_line_trace_correction = limit_float(
        right_line_trace_correction,
        -LINE_TRACE_MAX_CORRECTION,
        LINE_TRACE_MAX_CORRECTION);

    right_line_trace_previous_error = error;
    right_line_trace_last_sample_tick = now;
  }

  move_degree(
      body_degree,
      (vx >= 0.0f) ? 1.0f : -1.0f,
      right_line_trace_correction,
      drive_power);
  return true;
}

static bool right_line_trace_point_search(
    float body_degree,
    float vx,
    float drive_power)
{
  if (point_state == POINT_STATE_CANDIDATE)
  {
    right_line_trace_reset();
    move_degree(body_degree, 0.0f, 0.0f, 0.0f);
    return true;
  }

  return right_line_trace_move(body_degree, vx, drive_power);
}

static uint32_t body_turn_stable_tick = 0U;
static uint32_t body_turn_last_control_tick = 0U;

static void body_turn_reset(void)
{
  body_turn_stable_tick = 0U;
  body_turn_last_control_tick = 0U;
  stop_drive_motors();
}

/* In-place yaw turn with enough minimum PWM to start all four drive motors. */
static bool body_turn_update(float target_degree, uint32_t now)
{
  if ((body_turn_last_control_tick != 0U) &&
      ((now - body_turn_last_control_tick) < MOVE_CONTROL_PERIOD_MS))
  {
    return false;
  }
  body_turn_last_control_tick = now;

  bno055_euler_t euler;
  if (bno055_euler(&bno, &euler) != BNO_OK)
  {
    body_turn_stable_tick = 0U;
    stop_drive_motors();
    return false;
  }

  float error = normalize_degree(target_degree - euler.yaw);
  if (fabsf(error) <= BODY_TURN_TOLERANCE_DEG)
  {
    stop_drive_motors();
    if (body_turn_stable_tick == 0U)
    {
      body_turn_stable_tick = now;
    }
    return (now - body_turn_stable_tick) >= BODY_TURN_STABLE_MS;
  }

  body_turn_stable_tick = 0U;
  float turn_pwm = fabsf(body_turn_P_gain * error);
  turn_pwm = limit_float(turn_pwm, BODY_TURN_MIN_PWM, BODY_TURN_MAX_PWM);
  if (error < 0.0f)
  {
    turn_pwm = -turn_pwm;
  }

  /* Same rotation signs as move_degree(): positive error is positive yaw. */
  set_motor_pwm(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, -turn_pwm); /* M1 RR */
  set_motor_pwm(&htim3, TIM_CHANNEL_4, TIM_CHANNEL_3,  turn_pwm); /* M2 RL */
  set_motor_pwm(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1,  turn_pwm); /* M3 FL */
  set_motor_pwm(&htim2, TIM_CHANNEL_4, TIM_CHANNEL_3, -turn_pwm); /* M4 FR */
  return false;
}

static uint32_t bridge_imu_last_sample_tick = 0U;
static float bridge_last_pitch = 0.0f;
static bool bridge_pitch_sample_valid = false;

static void bridge_imu_reset(void)
{
  bridge_imu_last_sample_tick = 0U;
  bridge_last_pitch = 0.0f;
  bridge_pitch_sample_valid = false;
}

static bool bridge_sample_pitch(uint32_t now, float *pitch)
{
  if (bridge_pitch_sample_valid &&
      ((now - bridge_imu_last_sample_tick) < BRIDGE_IMU_SAMPLE_MS))
  {
    *pitch = bridge_last_pitch;
    return true;
  }

  bno055_euler_t euler;
  if (bno055_euler(&bno, &euler) != BNO_OK)
  {
    bridge_pitch_sample_valid = false;
    return false;
  }

  bridge_last_pitch = euler.pitch;
  bridge_imu_last_sample_tick = now;
  bridge_pitch_sample_valid = true;
  *pitch = bridge_last_pitch;
  return true;
}

/*
 * Move the mecanum base while holding body_degree with the BNO055 yaw.
 *
 * Angles are in degrees:
 *   body_degree : target absolute body angle (BNO055 yaw reference)
 *   move_degree : robot-relative travel direction, 0=forward, +90=right
 *
 * drive_power is the translation magnitude [0..MOTOR_PWM_LIMIT].  The
 * P_gain/I_gain/D_gain variables below control the yaw correction and are
 * intentionally left available for manual tuning.
 *
 * Motor mapping (positive command means forward):
 *   M1 RR = y + x - rot       M2 RL = y - x + rot
 *   M3 FL = y + x + rot       M4 FR = y - x - rot
 */
typedef enum
{
  AUTO_STATE_MOVE_RIGHT_TO_LINE = 0,
  AUTO_STATE_ALIGN_FRONT_LINE,
  AUTO_STATE_MOVE_FORWARD_TO_POINT,
  AUTO_STATE_ALIGN_POINT,
  AUTO_STATE_POINT_REACHED,
  AUTO_STATE_MOVE_TO_DONUT,
  AUTO_STATE_DONUT_GUIDE_APPROACH,
  AUTO_STATE_LOWER_ARM,
  AUTO_STATE_CLOSE_ARM,
  AUTO_STATE_DONUT_REACHED,
  AUTO_STATE_RAISE_ARM_HALF,
  AUTO_STATE_ROUTE_LEAVE_DONUT_POINT,
  AUTO_STATE_ROUTE_FORWARD_TO_POINT,
  AUTO_STATE_ROUTE_ALIGN_FORWARD_POINT,
  AUTO_STATE_ROUTE_LEFT_TO_POINT,
  AUTO_STATE_ROUTE_ALIGN_LEFT_POINT,
  AUTO_STATE_ROUTE_FORWARD_COUNT_POINTS,
  AUTO_STATE_ROUTE_ALIGN_FINAL_POINT,
  AUTO_STATE_ROUTE_TURN_RIGHT_AT_P5,
  AUTO_STATE_ROUTE_LEAVE_P5,
  AUTO_STATE_ROUTE_TRACE_TO_P7,
  AUTO_STATE_ROUTE_LEAVE_P7,
  AUTO_STATE_BRIDGE_FIND_UP_SLOPE,
  AUTO_STATE_BRIDGE_FIND_CREST,
  AUTO_STATE_BRIDGE_TOP_FORWARD,
  AUTO_STATE_BRIDGE_STOP_BEFORE_TURN,
  AUTO_STATE_BRIDGE_TURN_180,
  AUTO_STATE_BRIDGE_REVERSE_FIND_SLOPE,
  AUTO_STATE_BRIDGE_REVERSE_FIND_FLAT,
  AUTO_STATE_ROUTE_REVERSE_TO_P8,
  AUTO_STATE_ROUTE_ALIGN_P8,
  AUTO_STATE_P8_TURN_180,
  AUTO_STATE_P8_RAISE_ARM_TOP,
  AUTO_STATE_P8_FORWARD_TRACE,
  AUTO_STATE_P8_PREOPEN_REVERSE,
  AUTO_STATE_P8_OPEN_ARM,
  AUTO_STATE_P8_REVERSE_TO_POINT,
  AUTO_STATE_P8_FINAL_ALIGN,
  AUTO_STATE_P8_LEAVE_RIGHT,
  AUTO_STATE_P8_RIGHT_TO_P9,
  AUTO_STATE_P9_ALIGN,
  AUTO_STATE_P9_DIAGONAL_LEAVE,
  AUTO_STATE_P9_DIAGONAL_FIND_LINE,
  AUTO_STATE_P9_REVERSE_TRACE_REARM,
  AUTO_STATE_P9_REVERSE_TO_P10,
  AUTO_STATE_P10_FINAL_ALIGN,
  AUTO_STATE_ROUTE_COMPLETE,
  AUTO_STATE_ERROR
} auto_state_t;

static auto_state_t auto_state = AUTO_STATE_MOVE_RIGHT_TO_LINE;
static uint32_t auto_state_tick = 0U;
static uint32_t front_line_stable_tick = 0U;
static uint32_t front_line_detect_candidate_tick = 0U;
static uint32_t point_align_stable_tick = 0U;
static float point_align_integral_x = 0.0f;
static float point_align_integral_y = 0.0f;
static float point_align_previous_x = 0.0f;
static float point_align_previous_y = 0.0f;
static uint32_t point_align_last_tick = 0U;
static uint32_t donut_surface_candidate_tick = 0U;
static uint32_t gripper_hold_candidate_tick = 0U;
static uint32_t gripper_open_candidate_tick = 0U;
static uint32_t route_depart_clear_tick = 0U;
static uint32_t p9_diagonal_start_tick = 0U;
static uint8_t route_final_forward_point_count = 0U;
static uint8_t route_p5_to_p7_point_count = 0U;
static float bridge_flat_pitch = 0.0f;
static bool bridge_flat_pitch_valid = false;
static uint32_t bridge_slope_candidate_tick = 0U;
static uint32_t bridge_flat_candidate_tick = 0U;

static void apply_minimum_motion_vector(float *vx, float *vy)
{
  float vector_size = sqrtf((*vx * *vx) + (*vy * *vy));
  float minimum_vector = POINT_MIN_START_PWM / POINT_ALIGN_POWER;

  if (vector_size <= 0.001f)
  {
    *vx = 0.0f;
    *vy = 0.0f;
    return;
  }

  if (vector_size > 1.0f)
  {
    *vx /= vector_size;
    *vy /= vector_size;
    vector_size = 1.0f;
  }

  if (vector_size < minimum_vector)
  {
    float scale = minimum_vector / vector_size;
    *vx *= scale;
    *vy *= scale;
  }
}

static void point_align_reset(void)
{
  point_align_stable_tick = 0U;
  point_align_integral_x = 0.0f;
  point_align_integral_y = 0.0f;
  point_align_previous_x = 0.0f;
  point_align_previous_y = 0.0f;
  point_align_last_tick = 0U;
}

typedef enum
{
  DONUT_MOVE_RUNNING = 0,
  DONUT_MOVE_REACHED,
  DONUT_MOVE_ERROR
} donut_move_result_t;

static uint16_t get_selected_donut_tof(void)
{
  return (donut_tof_source == 0U) ? TOF_arm : TOF_hall;
}

/*
 * Move forward from the point until the selected downward-facing TOF sees
 * the configured donut surface distance.  This is a non-blocking task:
 * call it once per main-loop iteration.  The floor distance is used to
 * verify that the detected surface is actually closer than the floor.
 */
static donut_move_result_t DonutMove_Task(
    uint16_t floor_distance_mm,
    uint16_t surface_distance_mm)
{
  uint32_t now = HAL_GetTick();

  if ((floor_distance_mm == 0U) ||
      (surface_distance_mm == 0U) ||
      ((uint32_t)surface_distance_mm + DONUT_MIN_DISTANCE_DROP_MM >=
       (uint32_t)floor_distance_mm))
  {
    donut_surface_candidate_tick = 0U;
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
    return DONUT_MOVE_ERROR;
  }

  if ((tof_last_update_tick == 0U) ||
      ((now - tof_last_update_tick) > DONUT_TOF_STALE_MS))
  {
    donut_surface_candidate_tick = 0U;
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
    return DONUT_MOVE_RUNNING;
  }

  uint16_t tof_distance_mm = get_selected_donut_tof();
  if (tof_distance_mm == 0U)
  {
    donut_surface_candidate_tick = 0U;
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
    return DONUT_MOVE_RUNNING;
  }

  int32_t surface_error = (int32_t)tof_distance_mm -
                          (int32_t)surface_distance_mm;
  if (surface_error < 0)
  {
    surface_error = -surface_error;
  }

  bool closer_than_floor =
      ((int32_t)floor_distance_mm - (int32_t)tof_distance_mm) >=
      (int32_t)DONUT_MIN_DISTANCE_DROP_MM;
  bool near_donut_surface =
      surface_error <= (int32_t)donut_tof_tolerance_mm;

  if (closer_than_floor && near_donut_surface)
  {
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);

    if (donut_surface_candidate_tick == 0U)
    {
      donut_surface_candidate_tick = now;
    }

    if ((now - donut_surface_candidate_tick) >= DONUT_TOF_CONFIRM_MS)
    {
      return DONUT_MOVE_REACHED;
    }

    return DONUT_MOVE_RUNNING;
  }

  donut_surface_candidate_tick = 0U;
  move_degree(target_body_degree, 0.0f, 1.0f, DONUT_MOVE_POWER);
  return DONUT_MOVE_RUNNING;
}

/* Instantaneous front-line result used while already in the alignment state. */
static bool front_line_raw_is_present_with_confidence(float min_confidence)
{
  line_sensor_features_t front_features;
  line_sensor_position_t front;

  if ((line_adc_center_last_update_tick == 0U) ||
      ((HAL_GetTick() - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
  {
    return false;
  }

  get_line_sensor_position(
      line_adc_center,
      line_front_white_mean,
      line_front_white_sigma,
      line_front_green_mean,
      line_front_green_sigma,
      line_front_red_mean,
      line_front_red_sigma,
      &front);

  get_line_sensor_features(
      line_adc_center,
      line_front_white_mean,
      line_front_white_sigma,
      line_front_green_mean,
      line_front_green_sigma,
      line_front_red_mean,
      line_front_red_sigma,
      &front_features);

  return front_line_features_is_present(&front_features) &&
         (front.confidence >= min_confidence);
}

static bool front_line_raw_is_present(void)
{
  return front_line_raw_is_present_with_confidence(0.0f);
}

/*
 * Confirm the front line over time before allowing the automatic flow to
 * change state.  This prevents a single noisy CDC packet from immediately
 * entering AUTO_STATE_ALIGN_FRONT_LINE.
 */
static bool front_line_is_present_for_with_confidence(
    uint32_t confirm_ms,
    float min_confidence)
{
  uint32_t now = HAL_GetTick();

  if (!front_line_raw_is_present_with_confidence(min_confidence))
  {
    front_line_detect_candidate_tick = 0U;
    return false;
  }

  if (front_line_detect_candidate_tick == 0U)
  {
    front_line_detect_candidate_tick = now;
  }

  return (now - front_line_detect_candidate_tick) >= confirm_ms;
}

static bool front_line_is_present_for(uint32_t confirm_ms)
{
  return front_line_is_present_for_with_confidence(confirm_ms, 0.0f);
}

static bool front_line_is_present(void)
{
  return front_line_is_present_for(FRONT_LINE_CONFIRM_MS);
}

/*
 * Debug indication for line geometry:
 *   green = front/vertical line only
 *   blue  = right/horizontal line only
 *   red   = both sensors, i.e. intersection
 *   off   = no detected line
 *
 * This deliberately uses the same configured consecutive-sensor rule as the
 * motion flow, so the LED state shows the condition that can actually trigger
 * a state transition.
 */
static void LineDebugLED_Task(void)
{
  line_sensor_features_t front;
  line_sensor_features_t right;
  bool front_line = false;
  bool right_line = false;
  uint32_t now = HAL_GetTick();

  if ((line_adc_center_last_update_tick != 0U) &&
      ((now - line_adc_center_last_update_tick) <= LINE_CENTER_STALE_MS))
  {
    get_line_sensor_features(
        line_adc_center,
        line_front_white_mean,
        line_front_white_sigma,
        line_front_green_mean,
        line_front_green_sigma,
        line_front_red_mean,
        line_front_red_sigma,
        &front);
    front_line = front_line_features_is_present(&front);
  }

  get_line_sensor_features(
      line_adc_right,
      line_right_white_mean,
      line_right_white_sigma,
      line_right_green_mean,
      line_right_green_sigma,
      NULL,
      NULL,
      &right);
  right_line = right_line_features_is_present(&right);

  HAL_GPIO_WritePin(
      LED_G_GPIO_Port,
      LED_G_Pin,
      (front_line && !right_line) ? POINT_LED_ON_LEVEL : POINT_LED_OFF_LEVEL);
  HAL_GPIO_WritePin(
      LED_B_GPIO_Port,
      LED_B_Pin,
      (right_line && !front_line) ? POINT_LED_ON_LEVEL : POINT_LED_OFF_LEVEL);
  HAL_GPIO_WritePin(
      LED_R_GPIO_Port,
      LED_R_Pin,
      (front_line && right_line) ? POINT_LED_ON_LEVEL : POINT_LED_OFF_LEVEL);
}

static bool front_line_align_update(uint32_t now)
{
  line_sensor_position_t front;

  if (!front_line_raw_is_present())
  {
    front_line_stable_tick = 0U;
    move_degree(0.0f, 0.0f, 0.0f, 0.0f);
    return false;
  }

  get_line_sensor_position(
      line_adc_center,
      line_front_white_mean,
      line_front_white_sigma,
      line_front_green_mean,
      line_front_green_sigma,
      line_front_red_mean,
      line_front_red_sigma,
      &front);

  /* Front S0 is right and S7 is left: positive error means move right. */
  float error_x = FRONT_LINE_CENTER_POSITION - front.position;

  if (fabsf(error_x) <= POINT_ALIGN_ERROR_X)
  {
    move_degree(0.0f, 0.0f, 0.0f, 0.0f);

    if (front_line_stable_tick == 0U)
    {
      front_line_stable_tick = now;
    }

    return (now - front_line_stable_tick) >= FRONT_LINE_FOUND_STABLE_MS;
  }

  front_line_stable_tick = 0U;

  float vx = point_align_P_gain_x * error_x;
  float vector_size = fabsf(vx);
  if (vector_size > 1.0f)
  {
    vx = (vx > 0.0f) ? 1.0f : -1.0f;
  }
  else if (vector_size > 0.001f)
  {
    float minimum_vector = POINT_MIN_START_PWM / POINT_ALIGN_POWER;
    if (vector_size < minimum_vector)
    {
      vx = (vx > 0.0f) ? minimum_vector : -minimum_vector;
    }
  }

  move_degree(0.0f, vx, 0.0f, POINT_ALIGN_POWER);
  return false;
}

static bool point_align_update(uint32_t now)
{
  line_sensor_position_t front;
  line_sensor_position_t right;

  if ((line_adc_center_last_update_tick == 0U) ||
      ((now - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
  {
    point_align_stable_tick = 0U;
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
    return false;
  }

  get_line_sensor_position(
      line_adc_center,
      line_front_white_mean,
      line_front_white_sigma,
      line_front_green_mean,
      line_front_green_sigma,
      line_front_red_mean,
      line_front_red_sigma,
      &front);

  get_line_sensor_position(
      line_adc_right,
      line_right_white_mean,
      line_right_white_sigma,
      line_right_green_mean,
      line_right_green_sigma,
      line_right_red_mean,
      line_right_red_sigma,
      &right);

  if (!front.valid || !right.valid)
  {
    point_align_stable_tick = 0U;
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
    return false;
  }

  /* Front S0 is right and S7 is left: positive error means move right. */
  float error_x = FRONT_LINE_CENTER_POSITION - front.position;

  /* Right S0 is rear/lower and S7 is front/upper: positive means move forward. */
  float error_y = right.position - RIGHT_LINE_CENTER_POSITION;

  float dt = (point_align_last_tick == 0U) ?
             0.01f : ((float)(now - point_align_last_tick) / 1000.0f);
  if (dt < 0.001f)
  {
    dt = 0.001f;
  }
  point_align_last_tick = now;

  point_align_integral_x += error_x * dt;
  point_align_integral_y += error_y * dt;
  point_align_integral_x = limit_float(point_align_integral_x, -3.0f, 3.0f);
  point_align_integral_y = limit_float(point_align_integral_y, -3.0f, 3.0f);

  float derivative_x = (error_x - point_align_previous_x) / dt;
  float derivative_y = (error_y - point_align_previous_y) / dt;
  point_align_previous_x = error_x;
  point_align_previous_y = error_y;

  float vx = (point_align_P_gain_x * error_x) +
             (point_align_I_gain_x * point_align_integral_x) +
             (point_align_D_gain_x * derivative_x);
  float vy = (point_align_P_gain_y * error_y) +
             (point_align_I_gain_y * point_align_integral_y) +
             (point_align_D_gain_y * derivative_y);

  if ((fabsf(error_x) <= POINT_ALIGN_ERROR_X) &&
      (fabsf(error_y) <= POINT_ALIGN_ERROR_Y))
  {
    move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);

    if (point_align_stable_tick == 0U)
    {
      point_align_stable_tick = now;
    }

    return (now - point_align_stable_tick) >= POINT_ALIGN_STABLE_MS;
  }

  point_align_stable_tick = 0U;
  apply_minimum_motion_vector(&vx, &vy);
  move_degree(target_body_degree, vx, vy, POINT_ALIGN_POWER);
  return false;
}

void AutoControl_Task(void)
{
  uint32_t now = HAL_GetTick();

  switch (auto_state)
  {
    case AUTO_STATE_MOVE_RIGHT_TO_LINE:
      point_arrived = false;
      point_detection_enabled = false;

      if ((line_adc_center_last_update_tick == 0U) ||
          ((now - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_state_tick = 0U;
        break;
      }

      if (auto_state_tick == 0U)
      {
        auto_state_tick = now;
      }

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      /* First move right while holding body angle 0 degrees. */
      move_degree(0.0f, 1.0f, 0.0f, POINT_SEARCH_POWER);

      if (front_line_is_present())
      {
        front_line_stable_tick = now;
        auto_state_tick = now;
        auto_state = AUTO_STATE_ALIGN_FRONT_LINE;
      }
      break;

    case AUTO_STATE_ALIGN_FRONT_LINE:
      point_detection_enabled = false;

      if ((now - auto_state_tick) > FRONT_LINE_ALIGN_TIMEOUT_MS)
      {
        move_degree(0.0f, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if (front_line_align_update(now))
      {
        move_degree(0.0f, 0.0f, 0.0f, 0.0f);
        point_align_reset();
        PointDetect_Reset();
        point_detection_enabled = true;
        auto_state_tick = now;
        auto_state = AUTO_STATE_MOVE_FORWARD_TO_POINT;
      }
      break;

    case AUTO_STATE_MOVE_FORWARD_TO_POINT:
      point_detection_enabled = true;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        move_degree(0.0f, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      /* The front line is centered; now approach the intersection forward. */
      point_search_move(0.0f, 0.0f, 1.0f);

      if (point_event)
      {
        move_degree(0.0f, 0.0f, 0.0f, 0.0f);
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ALIGN_POINT;
      }
      break;

    case AUTO_STATE_ALIGN_POINT:
      point_detection_enabled = true;

      if ((now - auto_state_tick) > POINT_ALIGN_TIMEOUT_MS)
      {
        move_degree(0.0f, 0.0f, 0.0f, 0.0f);
        point_align_reset();
        auto_state_tick = now;
        break;
      }

      if (point_align_update(now))
      {
        point_arrived = true;
        current_point_index++;
        move_degree(0.0f, 0.0f, 0.0f, 0.0f);
        arm_vertical_command = 0.0f;
        point_detection_enabled = false;
        donut_surface_candidate_tick = 0U;
        auto_state = AUTO_STATE_POINT_REACHED;
      }
      break;

    case AUTO_STATE_POINT_REACHED:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      move_degree(0.0f, 0.0f, 0.0f, 0.0f);

      /* Start the donut approach automatically once both distances are set. */
      if ((donut_floor_distance_mm > 0U) &&
          (donut_surface_distance_mm > 0U))
      {
        auto_state_tick = now;
        donut_surface_candidate_tick = 0U;
        auto_state = AUTO_STATE_MOVE_TO_DONUT;
      }
      break;

    case AUTO_STATE_MOVE_TO_DONUT:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;

      /* Raspberry Pi delivery can pause.  Wait safely for a fresh, nonzero
       * TOF value and resume automatically instead of latching ERROR. */
      if ((tof_last_update_tick == 0U) ||
          ((now - tof_last_update_tick) > DONUT_TOF_STALE_MS) ||
          (get_selected_donut_tof() == 0U))
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        donut_surface_candidate_tick = 0U;
        auto_state_tick = now;
        break;
      }

      if ((now - auto_state_tick) > DONUT_MOVE_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      switch (DonutMove_Task(
          donut_floor_distance_mm,
          donut_surface_distance_mm))
      {
        case DONUT_MOVE_REACHED:
          move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
          auto_state_tick = now;
          auto_state = AUTO_STATE_DONUT_GUIDE_APPROACH;
          break;

        case DONUT_MOVE_ERROR:
          move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
          auto_control_error = true;
          auto_state = AUTO_STATE_ERROR;
          break;

        case DONUT_MOVE_RUNNING:
        default:
          break;
      }
      break;

    case AUTO_STATE_DONUT_GUIDE_APPROACH:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;

      /* Move slowly for the tunable distance to press the donut into the guide. */
      if ((now - auto_state_tick) >= donut_guide_approach_time_ms)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_state_tick = now;
        auto_state = AUTO_STATE_LOWER_ARM;
        break;
      }

      move_degree(
          target_body_degree,
          0.0f,
          1.0f,
          DONUT_GUIDE_APPROACH_POWER);
      break;

    case AUTO_STATE_LOWER_ARM:
      point_detection_enabled = false;
      move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);

      if (limit_bottom)
      {
        arm_vertical_command = 0.0f;
        arm_gripper_command = 0.0f;
        gripper_hold_candidate_tick = 0U;
        auto_state_tick = now;
        auto_state = AUTO_STATE_CLOSE_ARM;
        break;
      }

      if (limit_last_update_tick == 0U)
      {
        arm_vertical_command = 0.0f;
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, SET);
      
      /* Motor 6: positive power means down; stop at the bottom limit. */
      arm_vertical_command = ARM_LOWER_POWER;
      break;

    case AUTO_STATE_CLOSE_ARM:
      point_detection_enabled = false;
      move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
      arm_vertical_command = 0.0f;

      /* GET TOF order is "TOF <HALL> <ARM>".  Gripping uses the second
       * (ARM) value.  Never continue closing on missing or stale data. */
      if ((now - auto_state_tick) >= ARM_GRIPPER_CLOSE_TIMEOUT_MS)
      {
        arm_gripper_command = 0.0f;
        gripper_hold_candidate_tick = 0U;
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if ((tof_last_update_tick == 0U) ||
          ((now - tof_last_update_tick) > ARM_GRIPPER_TOF_STALE_MS) ||
          (TOF_arm == 0U))
      {
        arm_gripper_command = 0.0f;
        gripper_hold_candidate_tick = 0U;
        break;
      }

      if (gripper_hold_candidate_tick != 0U)
      {
        /* Keep the motor stopped while confirming.  Small rebound/noise up to
         * HOLD+HYSTERESIS must not restart the close motor. */
        arm_gripper_command = 0.0f;

        if (TOF_arm >
            (ARM_GRIPPER_HOLD_TOF_MM + ARM_GRIPPER_HYSTERESIS_MM))
        {
          gripper_hold_candidate_tick = 0U;
          break;
        }

        if ((now - gripper_hold_candidate_tick) >= ARM_GRIPPER_CONFIRM_MS)
        {
          auto_state = AUTO_STATE_DONUT_REACHED;
        }
        break;
      }

      if (TOF_arm <= ARM_GRIPPER_HOLD_TOF_MM)
      {
        /* Stop immediately at the first threshold crossing and start a
         * latched confirmation interval. */
        arm_gripper_command = 0.0f;
        gripper_hold_candidate_tick = now;
        break;
      }

      arm_gripper_command = -ARM_GRIPPER_CLOSE_POWER;
      break;

    case AUTO_STATE_DONUT_REACHED:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;
      move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
      auto_state_tick = now;
      auto_state = AUTO_STATE_RAISE_ARM_HALF;
      break;

    case AUTO_STATE_RAISE_ARM_HALF:
      point_detection_enabled = false;
      arm_gripper_command = 0.0f;
      move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);

      /* There is no arm-position sensor between the two limit switches, so
       * the half-height position is time based.  The top limit remains an
       * independent safety stop in ArmVertical_Move(). */
      if (limit_top ||
          ((now - auto_state_tick) >= ARM_HALF_RAISE_MS))
      {
        arm_vertical_command = 0.0f;
        point_arrived = false;
        route_depart_clear_tick = 0U;
        route_final_forward_point_count = 0U;
        point_align_reset();
        PointDetect_Reset();
        point_detection_enabled = false;
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_LEAVE_DONUT_POINT;
        break;
      }

      arm_vertical_command = -ARM_STARTUP_HOME_POWER;
      break;

    case AUTO_STATE_ROUTE_LEAVE_DONUT_POINT:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      /* Leave the pickup intersection before arming point detection.  If the
       * detector were reset while still centered on this point, the same
       * intersection could be counted as the next one. */
      move_degree(target_body_degree, 0.0f, 1.0f, POINT_SEARCH_POWER);

      if (point_candidate_is_present())
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (route_depart_clear_tick == 0U)
      {
        route_depart_clear_tick = now;
      }

      if ((now - route_depart_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        PointDetect_Reset();
        point_detection_enabled = true;
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_FORWARD_TO_POINT;
      }
      break;

    case AUTO_STATE_ROUTE_FORWARD_TO_POINT:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      point_search_move(target_body_degree, 0.0f, 1.0f);

      if (point_event)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_ALIGN_FORWARD_POINT;
      }
      break;

    case AUTO_STATE_ROUTE_ALIGN_FORWARD_POINT:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > ROUTE_ALIGN_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        current_point_index++;
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_LEFT_TO_POINT;
        break;
      }

      if (point_align_update(now))
      {
        current_point_index++;
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_LEFT_TO_POINT;
      }
      break;

    case AUTO_STATE_ROUTE_LEFT_TO_POINT:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      point_search_move(target_body_degree, -1.0f, 0.0f);

      if (point_event)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_ALIGN_LEFT_POINT;
      }
      break;

    case AUTO_STATE_ROUTE_ALIGN_LEFT_POINT:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > ROUTE_ALIGN_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        current_point_index++;
        route_final_forward_point_count = 0U;
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_FORWARD_COUNT_POINTS;
        break;
      }

      if (point_align_update(now))
      {
        current_point_index++;
        route_final_forward_point_count = 0U;
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_FORWARD_COUNT_POINTS;
      }
      break;

    case AUTO_STATE_ROUTE_FORWARD_COUNT_POINTS:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      /* Continue forward and stop at the third following intersection. */
      point_search_move_with_power(
          target_body_degree,
          0.0f,
          1.0f,
          (route_final_forward_point_count >=
           (ROUTE_FINAL_FORWARD_POINTS - 1U)) ?
              P5_POINT_SEARCH_POWER : POINT_SEARCH_POWER);

      if (point_event)
      {
        route_final_forward_point_count++;
        current_point_index++;
        auto_state_tick = now;

        if (route_final_forward_point_count >= ROUTE_FINAL_FORWARD_POINTS)
        {
          move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
          point_align_reset();
          auto_state = AUTO_STATE_ROUTE_ALIGN_FINAL_POINT;
        }
      }
      break;

    case AUTO_STATE_ROUTE_ALIGN_FINAL_POINT:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > ROUTE_ALIGN_TIMEOUT_MS)
      {
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        point_arrived = true;
        point_detection_enabled = false;
        point_align_reset();
        target_body_degree = normalize_degree(target_body_degree + 90.0f);
        route_p5_to_p7_point_count = 0U;
        bridge_flat_pitch_valid = false;
        body_turn_reset();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_TURN_RIGHT_AT_P5;
        break;
      }

      if (point_align_update(now))
      {
        point_arrived = true;
        move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
        point_detection_enabled = false;
        target_body_degree = normalize_degree(target_body_degree + 90.0f);
        route_p5_to_p7_point_count = 0U;
        bridge_flat_pitch_valid = false;
        body_turn_reset();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_TURN_RIGHT_AT_P5;
      }
      break;

    case AUTO_STATE_ROUTE_TURN_RIGHT_AT_P5:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BODY_TURN_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if (body_turn_update(target_body_degree, now))
      {
        float pitch;
        bridge_imu_reset();
        if (!bridge_sample_pitch(now, &pitch))
        {
          break;
        }

        /* Capture the actual mounting offset while P5 is still flat. */
        bridge_flat_pitch = pitch;
        bridge_flat_pitch_valid = true;
        route_depart_clear_tick = 0U;
        PointDetect_Reset();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_LEAVE_P5;
      }
      break;

    case AUTO_STATE_ROUTE_LEAVE_P5:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_move(target_body_degree, 1.0f);

      /* Do not arm the detector until the P5 intersection is behind us. */
      if (point_candidate_is_present())
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (route_depart_clear_tick == 0U)
      {
        route_depart_clear_tick = now;
      }
      if ((now - route_depart_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        PointDetect_Reset();
        point_detection_enabled = true;
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_TRACE_TO_P7;
      }
      break;

    case AUTO_STATE_ROUTE_TRACE_TO_P7:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_point_search(target_body_degree, 1.0f);

      if (point_event)
      {
        route_p5_to_p7_point_count++;
        current_point_index++;
        auto_state_tick = now;

        if (route_p5_to_p7_point_count >= ROUTE_P5_TO_P7_POINTS)
        {
          stop_drive_motors();
          point_detection_enabled = false;
          route_depart_clear_tick = 0U;
          line_trace_reset();
          auto_state = AUTO_STATE_ROUTE_LEAVE_P7;
        }
      }
      break;

    case AUTO_STATE_ROUTE_LEAVE_P7:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_move(target_body_degree, 1.0f);

      if (point_candidate_is_present())
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (route_depart_clear_tick == 0U)
      {
        route_depart_clear_tick = now;
      }
      if ((now - route_depart_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        bridge_slope_candidate_tick = 0U;
        bridge_flat_candidate_tick = 0U;
        bridge_imu_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_BRIDGE_FIND_UP_SLOPE;
      }
      break;

    case AUTO_STATE_BRIDGE_FIND_UP_SLOPE:
    {
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if (!bridge_flat_pitch_valid ||
          ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS))
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_move(target_body_degree, 1.0f);

      float pitch;
      if (!bridge_sample_pitch(now, &pitch))
      {
        bridge_slope_candidate_tick = 0U;
        break;
      }

      if (fabsf(pitch - bridge_flat_pitch) >= BRIDGE_SLOPE_ENTER_DEG)
      {
        if (bridge_slope_candidate_tick == 0U)
        {
          bridge_slope_candidate_tick = now;
        }
        if ((now - bridge_slope_candidate_tick) >= BRIDGE_SLOPE_CONFIRM_MS)
        {
          bridge_flat_candidate_tick = 0U;
          auto_state_tick = now;
          auto_state = AUTO_STATE_BRIDGE_FIND_CREST;
        }
      }
      else
      {
        bridge_slope_candidate_tick = 0U;
      }
      break;
    }

    case AUTO_STATE_BRIDGE_FIND_CREST:
    {
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_move(target_body_degree, 1.0f);

      float pitch;
      if (!bridge_sample_pitch(now, &pitch))
      {
        bridge_flat_candidate_tick = 0U;
        break;
      }

      if (fabsf(pitch - bridge_flat_pitch) <= BRIDGE_FLAT_RETURN_DEG)
      {
        if (bridge_flat_candidate_tick == 0U)
        {
          bridge_flat_candidate_tick = now;
        }
        if ((now - bridge_flat_candidate_tick) >= BRIDGE_FLAT_CONFIRM_MS)
        {
          auto_state_tick = now;
          auto_state = AUTO_STATE_BRIDGE_TOP_FORWARD;
        }
      }
      else
      {
        bridge_flat_candidate_tick = 0U;
      }
      break;
    }

    case AUTO_STATE_BRIDGE_TOP_FORWARD:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) >= BRIDGE_TOP_FORWARD_MS)
      {
        stop_drive_motors();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_BRIDGE_STOP_BEFORE_TURN;
        break;
      }

      (void)line_trace_move(target_body_degree, 1.0f);
      break;

    case AUTO_STATE_BRIDGE_STOP_BEFORE_TURN:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      /* Hold all four drive PWMs at zero before changing the yaw target. */
      stop_drive_motors();

      if ((now - auto_state_tick) >= BRIDGE_STOP_BEFORE_TURN_MS)
      {
        target_body_degree = normalize_degree(target_body_degree + 180.0f);
        body_turn_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_BRIDGE_TURN_180;
      }
      break;

    case AUTO_STATE_BRIDGE_TURN_180:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BODY_TURN_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if (body_turn_update(target_body_degree, now))
      {
        bridge_slope_candidate_tick = 0U;
        bridge_flat_candidate_tick = 0U;
        bridge_imu_reset();
        PointDetect_Reset();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_BRIDGE_REVERSE_FIND_SLOPE;
      }
      break;

    case AUTO_STATE_BRIDGE_REVERSE_FIND_SLOPE:
    {
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_move(target_body_degree, -1.0f);

      float pitch;
      if (!bridge_sample_pitch(now, &pitch))
      {
        bridge_slope_candidate_tick = 0U;
        break;
      }

      if (fabsf(pitch - bridge_flat_pitch) >= BRIDGE_SLOPE_ENTER_DEG)
      {
        if (bridge_slope_candidate_tick == 0U)
        {
          bridge_slope_candidate_tick = now;
        }
        if ((now - bridge_slope_candidate_tick) >= BRIDGE_SLOPE_CONFIRM_MS)
        {
          bridge_flat_candidate_tick = 0U;
          auto_state_tick = now;
          auto_state = AUTO_STATE_BRIDGE_REVERSE_FIND_FLAT;
        }
      }
      else
      {
        bridge_slope_candidate_tick = 0U;
      }
      break;
    }

    case AUTO_STATE_BRIDGE_REVERSE_FIND_FLAT:
    {
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_move(target_body_degree, -1.0f);

      float pitch;
      if (!bridge_sample_pitch(now, &pitch))
      {
        bridge_flat_candidate_tick = 0U;
        break;
      }

      if (fabsf(pitch - bridge_flat_pitch) <= BRIDGE_FLAT_RETURN_DEG)
      {
        if (bridge_flat_candidate_tick == 0U)
        {
          bridge_flat_candidate_tick = now;
        }
        if ((now - bridge_flat_candidate_tick) >= BRIDGE_FLAT_CONFIRM_MS)
        {
          PointDetect_Reset();
          point_detection_enabled = true;
          line_trace_reset();
          auto_state_tick = now;
          auto_state = AUTO_STATE_ROUTE_REVERSE_TO_P8;
        }
      }
      else
      {
        bridge_flat_candidate_tick = 0U;
      }
      break;
    }

    case AUTO_STATE_ROUTE_REVERSE_TO_P8:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_point_search(target_body_degree, -1.0f);

      if (point_event)
      {
        current_point_index++;
        stop_drive_motors();
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_ROUTE_ALIGN_P8;
      }
      break;

    case AUTO_STATE_ROUTE_ALIGN_P8:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > ROUTE_ALIGN_TIMEOUT_MS)
      {
        stop_drive_motors();
        point_arrived = true;
        point_detection_enabled = false;
        point_align_reset();
        target_body_degree = normalize_degree(target_body_degree + 180.0f);
        body_turn_reset();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_TURN_180;
        break;
      }

      if (point_align_update(now))
      {
        point_arrived = true;
        stop_drive_motors();
        point_detection_enabled = false;
        target_body_degree = normalize_degree(target_body_degree + 180.0f);
        body_turn_reset();
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_TURN_180;
      }
      break;

    case AUTO_STATE_P8_TURN_180:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BODY_TURN_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if (body_turn_update(target_body_degree, now))
      {
        stop_drive_motors();
        point_arrived = false;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_RAISE_ARM_TOP;
      }
      break;

    case AUTO_STATE_P8_RAISE_ARM_TOP:
      point_detection_enabled = false;
      arm_gripper_command = 0.0f;
      stop_drive_motors();

      /* Stop and wait if LIMIT delivery is interrupted.  Once packets resume,
       * restart the active-movement timeout from zero. */
      if ((limit_last_update_tick == 0U) ||
          ((now - limit_last_update_tick) > P8_LIMIT_STALE_MS))
      {
        arm_vertical_command = 0.0f;
        auto_state_tick = now;
        break;
      }

      if (limit_top)
      {
        arm_vertical_command = 0.0f;
        line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_FORWARD_TRACE;
        break;
      }

      if ((now - auto_state_tick) > P8_ARM_RAISE_TIMEOUT_MS)
      {
        arm_vertical_command = 0.0f;
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      arm_vertical_command = -ARM_STARTUP_HOME_POWER;
      break;

    case AUTO_STATE_P8_FORWARD_TRACE:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      /* Count only time in which fresh front-line data actually permits
       * movement, so a CDC pause cannot shorten the requested travel. */
      if (!line_trace_move(target_body_degree, 1.0f))
      {
        auto_state_tick = now;
        break;
      }

      if ((now - auto_state_tick) >= P8_FORWARD_TRACE_MS)
      {
        stop_drive_motors();
        line_trace_reset();
        gripper_open_candidate_tick = 0U;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_PREOPEN_REVERSE;
      }
      break;

    case AUTO_STATE_P8_PREOPEN_REVERSE:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) >= P8_PREOPEN_REVERSE_MS)
      {
        stop_drive_motors();
        gripper_open_candidate_tick = 0U;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_OPEN_ARM;
        break;
      }

      /* Move the chassis straight backward briefly without changing yaw. */
      move_degree(
          target_body_degree,
          0.0f,
          -1.0f,
          P8_PREOPEN_REVERSE_POWER);
      break;

    case AUTO_STATE_P8_OPEN_ARM:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      stop_drive_motors();

      /* Gripper opening is also measured by the second (ARM) TOF value. */

      if ((now - auto_state_tick) >= ARM_GRIPPER_OPEN_TIMEOUT_MS)
      {
        arm_gripper_command = 0.0f;
        gripper_open_candidate_tick = 0U;
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if ((tof_last_update_tick == 0U) ||
          ((now - tof_last_update_tick) > ARM_GRIPPER_TOF_STALE_MS) ||
          (TOF_arm == 0U))
      {
        arm_gripper_command = 0.0f;
        gripper_open_candidate_tick = 0U;
        break;
      }

      if (gripper_open_candidate_tick != 0U)
      {
        arm_gripper_command = 0.0f;

        if (((uint32_t)TOF_arm + ARM_GRIPPER_HYSTERESIS_MM) <
            ARM_GRIPPER_OPEN_TOF_MM)
        {
          gripper_open_candidate_tick = 0U;
          break;
        }

        if ((now - gripper_open_candidate_tick) >=
            ARM_GRIPPER_OPEN_CONFIRM_MS)
        {
          PointDetect_Reset();
          point_detection_enabled = true;
          line_trace_reset();
          arm_gripper_command = 0.0f;
          auto_state_tick = now;
          auto_state = AUTO_STATE_P8_REVERSE_TO_POINT;
        }
        break;
      }

      if (TOF_arm >= ARM_GRIPPER_OPEN_TOF_MM)
      {
        arm_gripper_command = 0.0f;
        gripper_open_candidate_tick = now;
        break;
      }

      arm_gripper_command = ARM_GRIPPER_OPEN_POWER;
      break;

    case AUTO_STATE_P8_REVERSE_TO_POINT:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > BRIDGE_STAGE_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_point_search(target_body_degree, -1.0f);

      if (point_event)
      {
        stop_drive_motors();
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_FINAL_ALIGN;
      }
      break;

    case AUTO_STATE_P8_FINAL_ALIGN:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > ROUTE_ALIGN_TIMEOUT_MS)
      {
        stop_drive_motors();
        point_arrived = true;
        point_detection_enabled = false;
        point_align_reset();
        route_depart_clear_tick = 0U;
        PointDetect_Reset();
        right_line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_LEAVE_RIGHT;
        break;
      }

      if (point_align_update(now))
      {
        point_arrived = true;
        stop_drive_motors();
        point_detection_enabled = false;
        point_align_reset();
        route_depart_clear_tick = 0U;
        PointDetect_Reset();
        right_line_trace_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_LEAVE_RIGHT;
      }
      break;

    case AUTO_STATE_P8_LEAVE_RIGHT:
      point_arrived = false;
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((line_adc_center_last_update_tick == 0U) ||
          ((now - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
      {
        stop_drive_motors();
        route_depart_clear_tick = 0U;
        auto_state_tick = now;
        break;
      }

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      /* Leave P8 slowly to the right while right-line position PID corrects
       * forward/rear drift and move_degree() holds the body yaw. */
      if (!right_line_trace_move(
              target_body_degree,
              1.0f,
              P9_POINT_SEARCH_POWER))
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (point_candidate_is_present())
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (route_depart_clear_tick == 0U)
      {
        route_depart_clear_tick = now;
      }

      if ((now - route_depart_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        PointDetect_Reset();
        point_detection_enabled = true;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P8_RIGHT_TO_P9;
      }
      break;

    case AUTO_STATE_P8_RIGHT_TO_P9:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)right_line_trace_point_search(
          target_body_degree,
          1.0f,
          P9_POINT_SEARCH_POWER);

      if (point_event)
      {
        stop_drive_motors();
        right_line_trace_reset();
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P9_ALIGN;
      }
      break;

    case AUTO_STATE_P9_ALIGN:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      /* Never continue to the diagonal leg when P9 alignment has failed. */
      if ((now - auto_state_tick) > P9_ALIGN_TIMEOUT_MS)
      {
        stop_drive_motors();
        point_detection_enabled = false;
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if (point_align_update(now))
      {
        current_point_index++;
        point_arrived = true;
        stop_drive_motors();
        point_detection_enabled = false;
        point_align_reset();
        route_depart_clear_tick = 0U;
        front_line_detect_candidate_tick = 0U;
        p9_diagonal_start_tick = now;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P9_DIAGONAL_LEAVE;
      }
      break;

    case AUTO_STATE_P9_DIAGONAL_LEAVE:
      point_arrived = false;
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((line_adc_center_last_update_tick == 0U) ||
          ((now - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
      {
        stop_drive_motors();
        route_depart_clear_tick = 0U;
        p9_diagonal_start_tick = now;
        auto_state_tick = now;
        break;
      }

      /* Right-rear diagonal while keeping exactly the same body heading. */
      move_degree(
          target_body_degree,
          P9_DIAGONAL_COMPONENT,
          -P9_DIAGONAL_COMPONENT,
          POINT_SEARCH_POWER);

      /* Do not mistake the front line at P9 for the line to be acquired. */
      if (front_line_raw_is_present())
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (route_depart_clear_tick == 0U)
      {
        route_depart_clear_tick = now;
      }

      if ((now - route_depart_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        front_line_detect_candidate_tick = 0U;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P9_DIAGONAL_FIND_LINE;
      }
      break;

    case AUTO_STATE_P9_DIAGONAL_FIND_LINE:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((line_adc_center_last_update_tick == 0U) ||
          ((now - line_adc_center_last_update_tick) > LINE_CENTER_STALE_MS))
      {
        stop_drive_motors();
        front_line_detect_candidate_tick = 0U;
        p9_diagonal_start_tick = now;
        auto_state_tick = now;
        break;
      }

      /* Ignore any vertical line reached too soon after leaving P9. */
      if ((p9_diagonal_start_tick == 0U) ||
          ((now - p9_diagonal_start_tick) < P9_EARLY_LINE_IGNORE_MS))
      {
        if (p9_diagonal_start_tick == 0U)
        {
          p9_diagonal_start_tick = now;
        }

        front_line_detect_candidate_tick = 0U;
        move_degree(
            target_body_degree,
            P9_DIAGONAL_COMPONENT,
            -P9_DIAGONAL_COMPONENT,
            POINT_SEARCH_POWER);
        break;
      }

      if (front_line_raw_is_present_with_confidence(
              P9_FRONT_LINE_MIN_CONFIDENCE))
      {
        /* Brake on the first raw hit and confirm it while stationary so the
         * diagonal search cannot coast completely across the guide line. */
        stop_drive_motors();

        if (front_line_is_present_for_with_confidence(
                P9_FRONT_LINE_CONFIRM_MS,
                P9_FRONT_LINE_MIN_CONFIDENCE))
        {
          PointDetect_Reset();
          line_trace_reset();
          route_depart_clear_tick = 0U;
          auto_state_tick = now;
          auto_state = AUTO_STATE_P9_REVERSE_TRACE_REARM;
        }
        break;
      }

      front_line_detect_candidate_tick = 0U;
      move_degree(
          target_body_degree,
          P9_DIAGONAL_COMPONENT,
          -P9_DIAGONAL_COMPONENT,
          POINT_SEARCH_POWER);
      break;

    case AUTO_STATE_P9_REVERSE_TRACE_REARM:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      if (!line_trace_move(target_body_degree, -1.0f))
      {
        route_depart_clear_tick = 0U;
        auto_state_tick = now;
        break;
      }

      /* Arm P10 detection only after the line-acquisition area is clear. */
      if (point_candidate_is_present())
      {
        route_depart_clear_tick = 0U;
        break;
      }

      if (route_depart_clear_tick == 0U)
      {
        route_depart_clear_tick = now;
      }

      if ((now - route_depart_clear_tick) >= POINT_REARM_CLEAR_MS)
      {
        PointDetect_Reset();
        point_detection_enabled = true;
        auto_state_tick = now;
        auto_state = AUTO_STATE_P9_REVERSE_TO_P10;
      }
      break;

    case AUTO_STATE_P9_REVERSE_TO_P10:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if ((now - auto_state_tick) > POINT_SEARCH_TIMEOUT_MS)
      {
        stop_drive_motors();
        auto_control_error = true;
        auto_state = AUTO_STATE_ERROR;
        break;
      }

      (void)line_trace_point_search(target_body_degree, -1.0f);

      if (point_event)
      {
        current_point_index++;
        stop_drive_motors();
        point_align_reset();
        auto_state_tick = now;
        auto_state = AUTO_STATE_P10_FINAL_ALIGN;
      }
      break;

    case AUTO_STATE_P10_FINAL_ALIGN:
      point_detection_enabled = true;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;

      if (((now - auto_state_tick) > ROUTE_ALIGN_TIMEOUT_MS) ||
          point_align_update(now))
      {
        point_arrived = true;
        stop_drive_motors();
        point_detection_enabled = false;
        point_align_reset();
        auto_state = AUTO_STATE_ROUTE_COMPLETE;
      }
      break;

    case AUTO_STATE_ROUTE_COMPLETE:
      point_detection_enabled = false;
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;
      move_degree(target_body_degree, 0.0f, 0.0f, 0.0f);
      break;

    case AUTO_STATE_ERROR:
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;
      stop_drive_motors();
      break;

    default:
      arm_vertical_command = 0.0f;
      arm_gripper_command = 0.0f;
      stop_drive_motors();
      auto_control_error = true;
      auto_state = AUTO_STATE_ERROR;
      break;
  }
}

void move_degree(float body_degree, float vx, float vy, float drive_power)
{
  static uint32_t last_tick = 0U;
  static float integral = 0.0f;
  static bool was_stopped = true;

  uint32_t now = HAL_GetTick();

  if ((last_tick != 0U) && ((now - last_tick) < MOVE_CONTROL_PERIOD_MS))
  {
    return;
  }

  float dt = (last_tick == 0U) ?
             ((float)MOVE_CONTROL_PERIOD_MS / 1000.0f) :
             ((float)(now - last_tick) / 1000.0f);
  last_tick = now;

  if (fabsf(drive_power) < 0.001f)
  {
    integral = 0.0f;
    PID_err = 0.0f;
    PID_err_old = 0.0f;
    was_stopped = true;
    stop_drive_motors();
    return;
  }

  bno055_euler_t euler;
  if (bno055_euler(&bno, &euler) != BNO_OK)
  {
    integral = 0.0f;
    stop_drive_motors();
    return;
  }

  float error = normalize_degree(body_degree - euler.yaw);

  if (was_stopped)
  {
    PID_err_old = error;
    integral = 0.0f;
    was_stopped = false;
  }

  integral += error * dt;
  integral = limit_float(integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  float derivative = (error - PID_err_old) / dt;
  PID_err = error;
  PID_err_old = error;
  degree_old = euler.yaw;

  float rotation = (P_gain * error) +
                   (I_gain * integral) +
                   (D_gain * derivative);

  float x = vx;
  float y = vy;
  float translation = limit_float(fabsf(drive_power), 0.0f, MOTOR_PWM_LIMIT);

  float m1 = translation * (y + x) - rotation;
  float m2 = translation * (y - x) + rotation;
  float m3 = translation * (y + x) + rotation;
  float m4 = translation * (y - x) - rotation;

  float maximum = fabsf(m1);
  if (fabsf(m2) > maximum) maximum = fabsf(m2);
  if (fabsf(m3) > maximum) maximum = fabsf(m3);
  if (fabsf(m4) > maximum) maximum = fabsf(m4);

  if (maximum > MOTOR_PWM_LIMIT)
  {
    float scale = MOTOR_PWM_LIMIT / maximum;
    m1 *= scale;
    m2 *= scale;
    m3 *= scale;
    m4 *= scale;
  }

  set_motor_pwm(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, m1); /* M1 RR */
  set_motor_pwm(&htim3, TIM_CHANNEL_4, TIM_CHANNEL_3, m2); /* M2 RL */
  set_motor_pwm(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, m3); /* M3 FL */
  set_motor_pwm(&htim2, TIM_CHANNEL_4, TIM_CHANNEL_3, m4); /* M4 FR */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  start_motor_pwm();
  
  bno.i2c = &hi2c1;
  bno.addr = BNO_ADDR_ALT;
  bno.mode = BNO_MODE_NDOF;

  error_bno bno_init_err = bno055_init(&bno);

  if (bno_init_err == BNO_OK)
  {
      bno055_set_unit(
          &bno,
          BNO_TEMP_UNIT_C,
          BNO_GYR_UNIT_DPS,
          BNO_ACC_UNITSEL_M_S2,
          BNO_EUL_UNIT_DEG
      );
  }

  HAL_ADC_Start_DMA(
    &hadc1,
    (uint32_t *)line_adc_right,
    8
  );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CDC_Protocol_Task();
    PointDetect_Task();

    /* Home the arm to the top limit before allowing the drive flow to start. */
    if (!arm_startup_home_done)
    {
      (void)ArmStartupHome_Task();
    }
    else
    {
      AutoControl_Task();
    }

    ArmVertical_Move(arm_vertical_command);
    ArmGripper_Move(arm_gripper_command);

    // if(TOF_arm > 50){
    //   HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SET);
    // }
    // else{
    //   HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, RESET);
    // }

    // if(TOF_hall > 50){
    //   HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SET);
    // }
    // else{
    //   HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, RESET);
    // }

    // if(limit_bottom | limit_top){
    //   HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, SET);
    // }
    // else{
    //   HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, RESET);
    // }

    /* RGB LED is reserved for line geometry debugging. */
    LineDebugLED_Task();
    // CDC_Debug_Task();


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1050-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1050-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1050-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1050-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC13_Pin|PC14_Pin|PC15_Pin|PC12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13_Pin PC14_Pin PC15_Pin PC12_Pin */
  GPIO_InitStruct.Pin = PC13_Pin|PC14_Pin|PC15_Pin|PC12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_SW_1_Pin Limit_SW_2_Pin */
  GPIO_InitStruct.Pin = Limit_SW_1_Pin|Limit_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
