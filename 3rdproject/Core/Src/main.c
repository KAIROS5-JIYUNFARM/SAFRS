/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <stdio.h>      // ★ 추가
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR  (0x68 << 1)   // I2C 7-bit -> left shift
#define MPU9250_WHOAMI 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ★ 추가: roll, pitch 변수
float roll = 0.0f;   // ★ 추가
float pitch = 0.0f;  // ★ 추가
float yaw = 0.0f;

float roll_offset = 0.0f;
float pitch_offset = 0.0f;

float roll_corrected = 0.0f;
float pitch_corrected = 0.0f;

// ★ PD 제어 변수
float Kp_roll = 2.1f;
float Kd_roll = 0.02f;
float Kp_pitch = 2.2f;
float Kd_pitch = 0.03f;
float Kp_yaw = 0.0f;
float Kd_yaw = 0.0f;

float roll_prev = 0.0f;
float pitch_prev = 0.0f;
float yaw_prev = 0.0f;

// ======================== ★ 추가: 필터 변수 ========================
float roll_f = 0.0f;
float pitch_f = 0.0f;
float yaw_f = 0.0f;

float comp_alpha = 0.97f;     // Roll/Pitch Complementary Filter
float yaw_lpf_alpha = 0.65f;  // Yaw LPF
float yaw_comp_alpha = 0.995f; // Yaw Drift Correction

// D 항 LPF
float roll_d = 0.0f;
float pitch_d = 0.0f;
float yaw_d = 0.0f;
float d_lpf_alpha = 0.97f;

// ★★★ 추가: 서보 출력 LPF 변수 ★★★
float servo_p_f = 90.0f;
float servo_r_f = 90.0f;
float servo_y_f = 90.0f;

float servo_p_offset = 0.0f;   // Pitch 서보 오프셋
float servo_r_offset = 0.0f;   // Roll 서보 오프셋
float servo_y_offset = 0.0f;   // Yaw 서보 오프셋

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// ★ 추가 함수 선언
// ★ constrain() 함수 프로토타입 (반드시 필요)
static float constrain(float val, float min_val, float max_val);
void MPU9250_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az);  // ★ 추가
void Calculate_RollPitch(int16_t ax, int16_t ay, int16_t az);
void MPU9250_Read_Gyro(int16_t *gx, int16_t *gy, int16_t *gz); // ★ 추가
// ---- 함수 프로토타입 추가 (중요!!) ----
uint8_t MPU9250_Read_WHOAMI(void);      // ★ 추가
void UART_Print(char *msg);
static void Servo_Write(uint32_t channel, float angle)
{
    if (angle > 180.0f) angle = 180.0f;
    if (angle < 0.0f) angle = 0.0f;
    uint16_t pulse = 500 + (angle * 2000) / 180;
    __HAL_TIM_SET_COMPARE(&htim3, channel, pulse);
}

// ======================== IMU & Servo 초기화 함수 ========================
void Gimbal_Init()
{
    // 서보 중앙
    Servo_Write(TIM_CHANNEL_1, 90);
    Servo_Write(TIM_CHANNEL_2, 90);
    Servo_Write(TIM_CHANNEL_3, 90);
    HAL_Delay(700);

    // IMU 10번 정도 읽어서 평균을 Offset으로 저장
    roll_offset = 0;
    pitch_offset = 0;

    for (int i = 0; i < 10; i++)
    {
        int16_t ax, ay, az;
        MPU9250_Read_Accel(&ax, &ay, &az);
        Calculate_RollPitch(ax, ay, az);

        roll_offset  += roll;
        pitch_offset += pitch;

        HAL_Delay(20);
    }

    roll_offset  /= 10.0f;
    pitch_offset /= 10.0f;

    // ---- 서보 오프셋(중앙 보정값) ----
    // 여기는 나중에 실제 기구물 평탄 맞춘 뒤 직접 측정해서 넣을 값
    servo_p_offset = 0.0f;   // 예: pitch가 오른쪽으로 기울면 +3.0 넣기
    servo_r_offset = 0.0f;   // 예: roll이 앞쪽으로 기울면 -2.5 넣기
    servo_y_offset = 0.0f;   // yaw는 대부분 0 유지

    // 초기화
    roll = pitch = yaw = 0;
    roll_f = pitch_f = yaw_f = 0;
    roll_prev = pitch_prev = yaw_prev = 0;

    servo_p_f = 90;
    servo_r_f = 90;
    servo_y_f = 90;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t id = MPU9250_Read_WHOAMI();
  if (id == 0x71)
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  else
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  // PWM 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // 초기 서보 중앙
  Servo_Write(TIM_CHANNEL_1, 90);
  Servo_Write(TIM_CHANNEL_2, 90);
  Servo_Write(TIM_CHANNEL_3, 90);
  HAL_Delay(500);
  // ★★★★★ 초기화 함수 호출 ★★★★★
  Gimbal_Init();

  uint32_t prev_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* -------------------- MPU9250 읽기 -------------------- */
	    int16_t ax, ay, az;
	    int16_t gx, gy, gz;

	    MPU9250_Read_Accel(&ax, &ay, &az);
	    MPU9250_Read_Gyro(&gx, &gy, &gz);

	    Calculate_RollPitch(ax, ay, az);
	    /* Gyro to dps */
	    float gyro_x = gx / 131.0f;
	    float gyro_y = gy / 131.0f;
	    float gyro_z_dps = gz / 131.0f;

	    /* -------------------- Δt 계산 -------------------- */
	    uint32_t now = HAL_GetTick();
	    float dt = (now - prev_tick) / 1000.0f;
	    if (dt <= 0) dt = 0.01f;
	    prev_tick = now;
	    /* Offset 보정 */
	    roll_corrected  = roll  - roll_offset;
	    pitch_corrected = pitch - pitch_offset;

	    // ================== ★(Complementary Filter) ==================
	    roll_f  = comp_alpha * (roll_f  + gyro_x * dt) + (1 - comp_alpha) * roll_corrected;
	    pitch_f = comp_alpha * (pitch_f + gyro_y * dt) + (1 - comp_alpha) * pitch_corrected;

	    // ================== ★ Yaw 보정 (Gyro-LPF + Drift) ==================
	    float yaw_raw = yaw_f + gyro_z_dps * dt;
	    yaw_f = yaw_lpf_alpha * yaw_f + (1 - yaw_lpf_alpha) * yaw_raw;
	    yaw_f = yaw_comp_alpha * yaw_f;

	    //   ★ 추가: IMU 2차 저역통과 필터 (노이즈 더 줄여 부드럽게)
	    // ==========================================================
	    static float roll_f2 = 0.0f, pitch_f2 = 0.0f;
	    float imu_lpf_alpha = 0.99f;  // 0.90 ~ 0.97 매우 추천

	    roll_f2  = imu_lpf_alpha * roll_f2  + (1 - imu_lpf_alpha) * roll_f;
	    pitch_f2 = imu_lpf_alpha * pitch_f2 + (1 - imu_lpf_alpha) * pitch_f;

	    // ------------------ PD 제어 입력 ------------------
	    float roll_err  = 0.0f - roll_f2;
	    float pitch_err = 0.0f - pitch_f2;
	    float yaw_err   = 0.0f - yaw_f;

	    // ★ 추가: 아주 작은 오차는 무시 (deadband)
	    float err_dead = 4.5;   // 2.5도 이내는 무시

	    if (fabsf(roll_err)  < err_dead) roll_err  = 0.0f;
	    if (fabsf(pitch_err) < err_dead) pitch_err = 0.0f;

	    // ================== ★ 모든 축 D항 LPF 적용 ==================
	    float roll_d_raw  = (roll_err  - roll_prev)  / dt;
	    float pitch_d_raw = (pitch_err - pitch_prev) / dt;
	    float yaw_d_raw   = (yaw_err   - yaw_prev)   / dt;

	    roll_d  = d_lpf_alpha * roll_d  + (1 - d_lpf_alpha) * roll_d_raw;
	    pitch_d = d_lpf_alpha * pitch_d + (1 - d_lpf_alpha) * pitch_d_raw;
	    yaw_d   = d_lpf_alpha * yaw_d   + (1 - d_lpf_alpha) * yaw_d_raw;

	    roll_prev  = roll_err;
	    pitch_prev = pitch_err;
	    yaw_prev   = yaw_err;

	    float roll_out  = Kp_roll  * roll_err  + Kd_roll  * roll_d;
	    float pitch_out = Kp_pitch * pitch_err + Kd_pitch * pitch_d;
	    float yaw_out   = 0.0f; // Kp_yaw   * yaw_err   + Kd_yaw   * yaw_d;

	    /* ★★★ 출력 제한 확대 (±60°) ★★★ */

	    float max_pid_roll  = 170.0f;
	    float max_pid_pitch = 170.0f;   // Pitch 크게
	    float max_pid_yaw   = 60.0f;  // ★ Yaw 회전 크게!

	    if (roll_out > max_pid_roll) roll_out = max_pid_roll;
	    if (roll_out < -max_pid_roll) roll_out = -max_pid_roll;

	    if (pitch_out > max_pid_pitch) pitch_out = max_pid_pitch;
	    if (pitch_out < -max_pid_pitch) pitch_out = -max_pid_pitch;

	    if (yaw_out > max_pid_yaw) yaw_out = max_pid_yaw;
	    if (yaw_out < -max_pid_yaw) yaw_out = -max_pid_yaw;

	            /* ---------- 서보 목표값 ---------- */
	    float servo_p_target = 90 + pitch_out + servo_p_offset;
	    float servo_r_target = 90 + roll_out  + servo_r_offset;
	    float servo_y_target = 90 + yaw_out   + servo_y_offset;
	    /* ---------- ★★★ Servo LPF 적용 ★★★ ---------- */
	    float servo_alpha = 0.92f;   // 0.7~0.9 추천

	    servo_p_f = servo_alpha * servo_p_f + (1 - servo_alpha) * servo_p_target;
	    servo_r_f = servo_alpha * servo_r_f + (1 - servo_alpha) * servo_r_target;
	    servo_y_f = servo_alpha * servo_y_f + (1 - servo_alpha) * servo_y_target;

	    /* ---------- ★★★ 서보 속도 제한 추가(여기 넣는다!) ★★★ */
	    float servo_speed_limit_p = 0.32f;   // Pitch
	    float servo_speed_limit_r = 0.32f;   // Roll
	    float servo_speed_limit_y = 4.0f;   // ★ Yaw는 빠르게! // 1 loop당 최대 3도만 이동

	    // diff는 목표 - 현재
	    float diff_p = servo_p_target - servo_p_f;
	    float diff_r = servo_r_target - servo_r_f;
	    float diff_y = servo_y_target - servo_y_f;

	    servo_p_f += constrain(diff_p, -servo_speed_limit_p, servo_speed_limit_p);
	    servo_r_f += constrain(diff_r, -servo_speed_limit_r, servo_speed_limit_r);
	    servo_y_f += constrain(diff_y, -servo_speed_limit_y, servo_speed_limit_y);

	    /* ======================================================
	        ★ 추가: 서보 데드존 (0.3도 이하 변화는 무시 → 떨림 완전히 제거)
	        ====================================================== */
	     static float prev_sp = 0, prev_sr = 0, prev_sy = 0;
	     float min_move = 0.47f;

	     if (fabsf(servo_p_f - prev_sp) < min_move) servo_p_f = prev_sp;
	     if (fabsf(servo_r_f - prev_sr) < min_move) servo_r_f = prev_sr;
	     if (fabsf(servo_y_f - prev_sy) < min_move) servo_y_f = prev_sy;

	     prev_sp = servo_p_f;
	     prev_sr = servo_r_f;
	     prev_sy = servo_y_f;

        /* ---------- 서보 제한 ---------- */
        if (servo_p_f > 180) servo_p_f = 180;
        if (servo_p_f <   0) servo_p_f =   0;

        if (servo_r_f > 180) servo_r_f = 180;
        if (servo_r_f <   0) servo_r_f =   0;

        if (servo_y_f > 180) servo_y_f = 180;
        if (servo_y_f <   0) servo_y_f =   0;


        /* ---------- 최종 서보 출력 ---------- */
        Servo_Write(TIM_CHANNEL_2, servo_p_f);
        Servo_Write(TIM_CHANNEL_3, servo_r_f);
        Servo_Write(TIM_CHANNEL_1, servo_y_f);


	    // ------------------ 디버그 ------------------
        char buf[150];
        sprintf(buf,
            "RollF:%.2f PitchF:%.2f YawF:%.2f | SP:%.1f SR:%.1f SY:%.1f\r\n",
            roll_f, pitch_f, yaw_f,
            servo_p_f, servo_r_f, servo_y_f
        );
        UART_Print(buf);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
// ★ constrian() 함수 추가 위치 ★
// ---------------------------
static float constrain(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

// ★ MPU9250 WHO_AM_I 레지스터(0x75)를 읽어오는 함수 정의
uint8_t MPU9250_Read_WHOAMI(void)
{
    uint8_t who = 0;
    HAL_I2C_Mem_Read(&hi2c1,          // I2C 핸들
                     MPU9250_ADDR,    // 슬레이브 주소
                     MPU9250_WHOAMI,  // WHO_AM_I 레지스터
                     I2C_MEMADD_SIZE_8BIT,
                     &who,
                     1,
                     100);
    return who;
}
// ★ 추가: ACCEL_X/Y/Z 읽기
void MPU9250_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t data[6];

    HAL_I2C_Mem_Read(&hi2c1,
                     MPU9250_ADDR,
                     0x3B,   // ACCEL_XOUT_H
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     6,
                     100);

    *ax = (int16_t)(data[0] << 8 | data[1]); // X
    *ay = (int16_t)(data[2] << 8 | data[3]); // Y
    *az = (int16_t)(data[4] << 8 | data[5]); // Z
}

void MPU9250_Read_Gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t d[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x43,
                     I2C_MEMADD_SIZE_8BIT, d, 6, 100);

    *gx = (int16_t)(d[0] << 8 | d[1]);
    *gy = (int16_t)(d[2] << 8 | d[3]);
    *gz = (int16_t)(d[4] << 8 | d[5]);
}

// ★ 추가: Roll / Pitch 계산
void Calculate_RollPitch(int16_t ax, int16_t ay, int16_t az)
{
    float ax_f = ax / 16384.0f;
    float ay_f = ay / 16384.0f;
    float az_f = az / 16384.0f;

    roll  = atan2(ay_f, az_f) * 57.2958f;
    pitch = atan2(-ax_f, sqrt(ay_f*ay_f + az_f*az_f)) * 57.2958f;
}



void UART_Print(char *msg)
{
    HAL_USART_Transmit(&husart2, (uint8_t*)msg, strlen(msg), 10);
}
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
