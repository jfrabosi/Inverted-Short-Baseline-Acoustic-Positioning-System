/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body.
 * @author         : Jakob Frabosilio
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "lis3mdl_reg.h"
#include "lsm6dsox_reg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float roll, pitch, yaw;
} EulerAngles;

typedef struct {
    float x, y, z;
} Point3D;

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    arm_matrix_instance_f32 x; // State estimate
    arm_matrix_instance_f32 P; // Estimate covariance
    arm_matrix_instance_f32 F; // State transition matrix
    arm_matrix_instance_f32 H; // Measurement matrix
    arm_matrix_instance_f32 Q; // Process noise covariance
    arm_matrix_instance_f32 R; // Measurement noise covariance
    arm_matrix_instance_f32 K; // Kalman gain

    float32_t *x_data;
    float32_t *P_data;
    float32_t *F_data;
    float32_t *H_data;
    float32_t *Q_data;
    float32_t *R_data;
    float32_t *K_data;

    size_t state_dim;
    size_t meas_dim;
} KalmanFilter;

typedef struct {
    uint32_t num_mics;
    Point3D base_points[];  // Flexible array member
} MicArray_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// for ADC and recording
#define ADC_BUF_LEN 2048			    							/**< number of samples to record from each mic during full recording state */
#define TRIG_BUF_LEN 64		        							/**< number of samples to record from each mic during trigger state */
#define TRIG_THRES 50			      								/**< uint16 threshold value for triggering transition from trigger state to full recording state */
#define NUM_THRES_ROW 5           							/**< number of values in a row that must exceed TRIG_THRES to trigger transition from trigger state to full recording state */
#define adc1conv(data) ((data) & 0xFFFF)				/**< conversion for extracting adc1 (mics 0 and 2) data from ADC in multimode, saved as uint32_t */
#define adc2conv(data) ((data >> 16) & 0xFFFF)	/**< conversion for extracting adc2 (mics 1 and 3) data from ADC in multimode, saved as uint32_t */

// for FFT and HJS position estimate
#define FFT_BUF_LEN ADC_BUF_LEN*2								/**< length of FFT buffers, twice the length of each mic buffer for zero-padding */
#define NUM_MIC 4			            							/**< number of microphones in mic array */
#define BASELINE_MM 250			      							/**< spacing from each microphone to center of array (assuming a circular pattern) in mm */
#define SOUND_SPEED_M_S	343.0f	  							/**< speed of sound in meters per second */

// for IMU and orientation
#define ARM_MATH_CM7			        							/**< using cortex M7 chip (STM32H723ZG) */
#define SENSOR_BUS hi2c1												/**< using I2C1 for comms with IMU */
#define sampleFreq 208.0f												/**< sample rate for accel/gyro, update frequency of Madgwick filter*/
#define beta 0.11f 															/**< beta hyperparameter for Madgwick filter */

// for data transmission over UART
#define START_MARKER '<'												/**< character that defines start of robust float message */
#define END_MARKER '>'													/**< character that defines end of robust float message */
#define MAX_MESSAGE_LENGTH 50										/**< max length of robust float message */
#define CHECKSUM_MODULUS 256										/**< checksum modulus for robust float message error correction */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

// for state tracking and flags
uint8_t state = 0;				                    		/**< state tracking variable */
static uint8_t imu_rdy = 0;                       /**< flag that sets to 1 when the IMU has been initialized */
static uint8_t data_rdy = 0;                      /**< flag that sets to 1 when data is ready to be read from IMU */
static uint8_t imu_calibrated = 0;                /**< flag that sets to 1 when the IMU gyroscope has been calibrated */
static uint8_t recording = 0;											/**< flag that sets to 1 when ADCs are recording, prevents Madgwick filter from updating to reduce system noise */
int imu_init_set = 0;                             /**< flag that sets to 1 when the filter has converged and an initial orientation has been set */
int acc_stable_cnt = 0;                           /**< number of filter updates that the accelerometer has been stable for */
int last_stable_i = 0;                            /**< last index that the IMU was stable at */
int acc_stable_flag = 0;                          /**< flag that sets to 1 when the accelerometer is stable */
uint32_t timestamp;                               /**< current time when the Madgwick filter is run */
uint32_t previousTimestamp;                       /**< previous time that the Madgwick filter was run */
float deltat = 0.0f;                              /**< amount of time in seconds since last Madgwick filter update was run */
uint32_t timestamp2;                              /**< current time when the acoustic position estimate is calculated */
uint32_t previousTimestamp2;                      /**< previous time that the acoustic position estimate was calculated */
float deltat_pos = 0.0f;                          /**< amount of time in seconds since last position estimate was calculated */

// for ADC and recording
uint32_t adc_buf[ADC_BUF_LEN*NUM_MIC/2];		    	/**< buffer to hold all microphone data from ADC during full recording state */
uint32_t trig_buf[TRIG_BUF_LEN*NUM_MIC/2];	    	/**< buffer to hold all microphone data from ADC during trigger state */

// for calculating average and standard deviation for each microphone
float mic0_avg = 0, mic1_avg = 0, mic2_avg = 0, mic3_avg = 0;
float mic0_var = 0, mic1_var = 0, mic2_var = 0, mic3_var = 0;
float mic0_stdev = 0, mic1_stdev = 0, mic2_stdev = 0, mic3_stdev = 0;

// for FFT, HJS position estimate, and Kalman filtering
arm_rfft_fast_instance_f32 fftHandler;		    		/**< handler for real FFT calculations */
float32_t measured_time_shifts[NUM_MIC-1];	  		/**< array to hold measured time shifts between mics (all relative to mic 0) */
float32_t calcd_time_shifts[NUM_MIC-1];	  				/**< array to hold calculated time shifts (HJS best estimate) between mics (all relative to mic 0) */
float32_t sound_speed_mps = SOUND_SPEED_M_S;			/**< speed of sound in meters per second */
float32_t baseline = BASELINE_MM / 1000.0f; 			/**< spacing from each microphone to center of array in meters */
Point3D kf_comb_pos_est;                          /**< best position estimate from combined Kalman filter */
Point3D kf_isbl_pos_est;                          /**< best position estimate from just acoustic position estimate Kalman filter */
Point3D raw_isbl_pos_est;                         /**< best position estimate from raw acoustic position estimate */

// for IMU and orientation (and dead reckoning)
static int16_t data_raw_acceleration[3];          /**< raw acceleration data from IMU */
static int16_t data_raw_angular_rate[3];          /**< raw gyroscope data from IMU */
static int16_t data_raw_magnetic[3];              /**< raw magnetometer data from IMU */
static int16_t data_raw_temperature;              /**< raw temperature data from IMU */
static float acceleration_g[3];                   /**< converted acceleration data in g's */
static float angular_rate_dps[3];                 /**< converted gyroscope data in deg/s */
static float magnetic_G[3];                       /**< converted magnetometer data in Gauss */
static float temperature_degC;                    /**< converted temperature data in degC */
static uint8_t whoamI, rst;                       /**< statuses for IMU initialization */
uint16_t mag_threshold = (uint16_t)(1711 * 0.192f * 0.5f);  /**< threshold for registering magnetometer data */
float g_cal[3] = {0.0f, 0.0f, 0.0f};              /**< calibration offsets for gyroscope */
int imu_i = -10000;                               /**< Madgwick filter iteration count, set to -(number of iterations before convergence) */
EulerAngles angles;                               /**< Euler angle representation of current orientation */
float init_yaw;                                   /**< initial yaw angle of system (once converged) in radians */
Point3D delta_x_imu;                              /**< change in position since last acoustic position estimate, as measured by IMU dead-reckoning */
Point3D v_corrected;                              /**< current velocity of IMU in global frame (orientation-corrected) */
Point3D v_corrected_last_stable;                  /**< corrected velocity of IMU from last time the accelerometer was stable, used for correcting drift */
Quaternion initQuat;                              /**< quaternion representation of initial orientation as Quaternion struct (once converged) */
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;      /**< quaternion representation of current orientation as floats */

// for data transmission over UART
float fossl_x;                                    /**< current x-coordinate of the iSBL array's center */
int bad_depth_cnt = 0;                            /**< counts the number of failed transmissions when trying to receive fossl_x value from ESP32 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_ADC1_Init(void);
static void MX_ETH_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM13_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

// for FFT and time differencing
float32_t calculate_time_shift(float32_t *micA_buf, float32_t *micB_buf);

// for generating and rotating mic array
void calculate_base_points(MicArray_t* mic_array, float baseline);
MicArray_t* rotateMicArray(MicArray_t* original_mic_array, Quaternion quat);
MicArray_t* createMicArray(uint32_t num_mics);
void freeMicArray(MicArray_t* mic_array);

// for HJS
float32_t calculate_residual(MicArray_t* mic_array, Point3D target_point, float32_t measured_time_shifts[]);
float32_t calculate_squared_diff(float32_t measured_data[], float32_t target_data[], int data_len);
float32_t calculate_toa(Point3D pointA, Point3D pointB, float32_t sound_speed_mps);
Point3D hooke_jeeves_search_2d(MicArray_t* mic_array, Point3D init_pos_est, float32_t measured_time_shifts[], int max_iter, float32_t min_residual, float32_t min_spacing, float32_t scale_factor);
Point3D hooke_jeeves_search_3d(MicArray_t* mic_array, Point3D init_pos_est, float32_t measured_time_shifts[], int max_iter, float32_t min_residual, float32_t min_spacing, float32_t scale_factor);
int arg_min(float32_t data_array[], int data_len);

// for reading from IMU
static int32_t lsm6dsox_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t lsm6dsox_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t lis3mdl_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t lis3mdl_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static stmdev_ctx_t dev_ctx;
static stmdev_ctx_t dev_ctx_m;
void lsm6dsox_read_data_drdy_handler(void);
void lsm6dsox_read_data_drdy(void);
void lis3mdl_read_data_drdy_handler(void);
void lis3mdl_read_data_drdy(void);

// for calibrating IMU and running Madgwick filter
void gyro_calibration(float g_cal[3], int num_iter);
void init_imu(void);
void update_filter(void);

// quaternion and misc helper functions
EulerAngles QuaternionToEulerAngles(Quaternion q);
Point3D rotatePoint(Point3D point, Quaternion q);
Quaternion EulerAnglesToQuaternion(EulerAngles angles);
Quaternion multiply_quaternions(Quaternion q1, Quaternion q2);
float extract_yaw(Quaternion q);
Quaternion create_yaw_quaternion(float yaw);
void normalize_quaternion(Quaternion* q);
float randn(float mean, float stddev);
float invSqrt(float x);

// Kalman filter functions
KalmanFilter* create_kalman_filter(size_t state_dim, size_t meas_dim);
void kalman_filter_init(KalmanFilter* kf, float32_t* x_values, float32_t* P_values,
                        float32_t* F_values, float32_t* H_values,
                        float32_t* Q_values, float32_t* R_values);
void kalman_filter_predict(KalmanFilter* kf);
void kalman_filter_update(KalmanFilter* kf, float32_t* measurement);
KalmanFilter* init_kalman_filter_combined();
KalmanFilter* init_kalman_filter_isbl();
void update_kalman_matrices(KalmanFilter* kf_combined, KalmanFilter* kf_isbl, float32_t dt);

// for data transmission over UART
void transmit_line(UART_HandleTypeDef *huart, const char *line);
HAL_StatusTypeDef receiveRobustFloat(UART_HandleTypeDef *huart, float *result);

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
	int stationary = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_UART4_Init();
  MX_TIM14_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // start timer for Madgwick filter (finds delta_t between filter updates)
  HAL_TIM_Base_Start(&htim13);

  // start timer for iSBL position update (finds delta_tpos between acoustic updates)
  HAL_TIM_Base_Start(&htim14);

  // run IMU initialization
  init_imu();

  // initialize Kalman filters
  KalmanFilter* kf_combined = init_kalman_filter_combined();
  KalmanFilter* kf_isbl = init_kalman_filter_isbl();

  // initialize FFT handler with correct FFT length
  arm_rfft_fast_init_f32(&fftHandler, FFT_BUF_LEN);

  // initialize the mic array base points
  MicArray_t* original_mic_array = createMicArray(NUM_MIC);
  calculate_base_points(original_mic_array, baseline);

  // go to first state
  state = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

    /**********************************************************************************
     * @brief STATE 0: INITIALIZATION STATE
     *
     * In the initialization state, the FFT handler is initialized and the system
     * transitions to the trigger state after starting DMA and starting the timer
     * controlling the ADC
     **********************************************************************************/
    if (state == 0){

      // small delay to let system settle/initialize
      HAL_Delay(1000);

      // start DMA from ADCs, writing TRIG_BUF_LEN samples to trig_buf (trigger state)
      // note: using multimode, which means that ADC2 is a slave to ADC1 - when ADC1 is triggered,
      // it simultaneously reads ADC2. this doubles the sampling rate that the system can achieve
      HAL_ADC_Start(&hadc2);
      HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)trig_buf, TRIG_BUF_LEN * NUM_MIC / 2);

      // start timer that controls ADCs; when timer updates, a new ADC value is recorded from each microphone
      HAL_TIM_Base_Start_IT(&htim3);

      // transition to trigger state
      state = 1;

    }

    /**********************************************************************************
     * @brief STATE 1: TRIGGERING STATE
     *
     * In the triggering state, we just wait until we transition to a different state.
     * The trigger process is handled in the callback function HAL_ADC_ConvCpltCallback
     * Basically, we are waiting until the microphones read five values in a row that
     * exceed the TRIG_THRES value defined in the private defines section of the code.
     **********************************************************************************/
    else if (state == 1){
    }

    /**********************************************************************************
     * @brief STATE 2: FULL RECORDING STATE
     *
     * In the full recording state, we just wait until we transition to a different
     * state. The full recording process is handled in the callback function
     * HAL_ADC_ConvCpltCallback. During this state, we just record the values of the
     * ADC to the adc_buf and wait until it's full.
     **********************************************************************************/
    else if (state == 2){
    }


    /**********************************************************************************
     * @brief STATE 3: RECORDING VALIDATION STATE
     *
     * In the recording validation state, we check if the long recording has enough
     * variance to produce good time shifts.
     *
     * First, we de-interleave the ADC data into individual microphone buffers. The raw
     * ADC data is saved like this:
     *   adc_buf[] = {mic02[0], mic13[0], mic02[1], mic13[1], ...}
     *   (assuming four microphones in this case)
     * Note that the ADCs are operating in MultiMode. In this mode, data is saved to
     * the buffer in uint32_t, where one half of each uint32_t is from ADC1 and the
     * other is from ADC2. We have to separate each uint32_t into two uint16_ts. So, the
     * first uint32_t is split into uint16_t mic0[0] and mic2[0], and similarly the next
     * is split into uint16_t mic1[0] and mic3[0]. The buffers it splits into are
     * determined by the hardware connections to the ADCs.
     *
     * During this step, we also prepare to normalize each microphone buffer's data. We
     * assume 12-bit ADC readings which range from 0 to 4095. We find the average and
     * standard deviation of the readings. We also cast the uint16 values as float32
     * values for better math-ing.
     *
     * Lastly, we check if the readings are "good." If the standard deviation of the
     * data from all four mics exceeds a threshold, then we assume the data has enough
     * variance to determine a good time shift.
     **********************************************************************************/
    else if (state == 3){

      // set recording flag to 0 because the ADC has stopped recording
      recording = 0;

      // stop the ADC trigger timer
      HAL_TIM_Base_Stop_IT(&htim3);

      // variables for calculating average and standard deviation for each microphone
      mic0_avg = 0, mic1_avg = 0, mic2_avg = 0, mic3_avg = 0;
      mic0_var = 0, mic1_var = 0, mic2_var = 0, mic3_var = 0;
      mic0_stdev = 0, mic1_stdev = 0, mic2_stdev = 0, mic3_stdev = 0;

      // calculate average of each mic signal
      for (int i = 0; i < ADC_BUF_LEN * NUM_MIC / 2; i += NUM_MIC / 2) {
        mic0_avg += (float)adc1conv(adc_buf[i]);
        mic1_avg += (float)adc1conv(adc_buf[i+1]);
        mic2_avg += (float)adc2conv(adc_buf[i]);
        mic3_avg += (float)adc2conv(adc_buf[i+1]);
      }

      mic0_avg /= (float)ADC_BUF_LEN;
      mic1_avg /= (float)ADC_BUF_LEN;
      mic2_avg /= (float)ADC_BUF_LEN;
      mic3_avg /= (float)ADC_BUF_LEN;

      // calculate variance of each mic
      for (int i = 0; i < ADC_BUF_LEN * NUM_MIC / 2; i += NUM_MIC / 2) {
        mic0_var += powf((float)adc1conv(adc_buf[i]) - mic0_avg, 2);
        mic1_var += powf((float)adc1conv(adc_buf[i+1]) - mic1_avg, 2);
        mic2_var += powf((float)adc2conv(adc_buf[i]) - mic2_avg, 2);
        mic3_var += powf((float)adc2conv(adc_buf[i+1]) - mic3_avg, 2);
      }

      mic0_var /= (float)(ADC_BUF_LEN - 1);
      mic1_var /= (float)(ADC_BUF_LEN - 1);
      mic2_var /= (float)(ADC_BUF_LEN - 1);
      mic3_var /= (float)(ADC_BUF_LEN - 1);

      // calculate standard deviation of each mic
      mic0_stdev = sqrtf(mic0_var);
      mic1_stdev = sqrtf(mic1_var);
      mic2_stdev = sqrtf(mic2_var);
      mic3_stdev = sqrtf(mic3_var);

      // check if each microphone exceeds the threshold for "good" data
      int stdev_above_thres = 0;
      stdev_above_thres += mic0_stdev > (TRIG_THRES);
      stdev_above_thres += mic1_stdev > (TRIG_THRES);
      stdev_above_thres += mic2_stdev > (TRIG_THRES);
      stdev_above_thres += mic3_stdev > (TRIG_THRES);

      // if the signal is good and the initial IMU angle has been set (remove yaw), then
      // move to the data processing state
      if ((stdev_above_thres >= NUM_MIC) && imu_init_set){
        state = 4;
      }
      else{
        // reset all ADC buffers
        memset(adc_buf, 0, sizeof(adc_buf));
        memset(trig_buf, 0, sizeof(trig_buf));

        // turn off LEDs for debugging
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_RESET);  // Turn off green LED
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);  // Turn off yellow LED
        HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);  // Turn off red LED
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);  // Turn off LED on Pin E2

        // return to the trigger state
        state = 1;

        // restart the DMA from ADCs and the timer that controls the ADCs
        HAL_ADC_Start(&hadc2);
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)trig_buf, TRIG_BUF_LEN * NUM_MIC / 2);
        HAL_TIM_Base_Start_IT(&htim3);

      }
    }

    /**********************************************************************************
     * @brief STATE 4: DATA PROCESSING STATE
     *
     * In the data processing state, we process the data (big surprise!). We assume
     * that we have "good data" as defined in State 3.
     *
     * First, we calculate the time shifts between each microphone and microphone 0. We
     * have a dedicated function for this that takes in two microphone buffers (for
     * whichever microphones we're trying to find the time shift between) and returns
     * the time shift between the signals in seconds. This function converts the time
     * data to the frequency domain using real FFTs, then computes the complex conjugate
     * of the second array (so that we calculate the cross-correlation of the two arrays
     * and not the convolution), then multiplies the two arrays and takes the inverse
     * real FFT of the product. This results in the same result as taking the direct
     * cross-correlation of the two arrays but in much less time; FFT cross-correlation
     * has time complexity O(n) = n*log(n), while direct cross-correlation has time
     * complexity O(n) = n^2. We then find the largest value in the cross-correlation
     * output and save its index - this value corresponds to the most likely time shift
     * between the two signals. We use the sampling frequency of the ADC to compute the
     * corresponding time shift between the two signals. We do this for all microphones
     * compared to microphone 0 (except microphone 0, because that tells us nothing
     * interesting).
     *
     * Next, we take the current orientation of the microphone array (as determined by
     * the Madgwick filter running on IMU data in the background) and rotate the
     * microphone array base points by it. This ensures that the current orientation
     * of the platform is being taken into account when running Hooke-Jeeves search.
     *
     * Then, using the measured time shifts, we find the best position estimate using
     * Hooke-Jeeves search. This algorithm searches in d-dimensional space for the
     * point that minimizes some residual function. In our case, we use the algorithm
     * like this:
     *   - Define an initial guess for the transmitter's XYZ coordinates
     *   - Generate a set of points that are adjacent to those XYZ coordinates along
     *     all three dimensions with a designated spacing
     *      - For point {X, Y, Z} and spacing S, it would generate the set {{X+S,Y,Z},
     *        {X-S,Y,Z}, {X,Y+S,Z}, ... {X,Y,Z-S}}
     *   - For each of the points in that set:
     *     - Calculate what the expected time shifts between microphones would be for a
     *       transmitter at that position
     *     - Compute the residual as the sum of absolute differences between the
     *       estimated and measured time shifts
     *   - Then, choose the point that gives the smallest residual and move in that
     *     direction. Repeat this step until the residual starts to increase relative
     *     to the previous value, then start from the "generate a set of points" step,
     *     dividing the current "spacing" value in half
     *
     * This algorithm allows us to find the most likely position for the transmitter.
     * It can get stuck in local minimums for some functions, but our residual function
     * tends to not have this problem. Due to the small baseline, the algorithm has
     * trouble with finding the true position of the transmitter but very reliably finds
     * a point that is along the line between the reciever array's center and the
     * transmitter. To help, we use "depth measurements" (simulated using known distance
     * data) to fix the Hooke-Jeeves search to a particular YZ plane. Then, we reflect
     * the relative position vector (HJS gives us transmitter position relative to the
     * array center) across the known position of the transmitter to give the acoustic
     * position estimate of the receiver array.
     *
     * Finally, we combine the acoustic position estimate with a dead-reckoning position
     * estimate from the IMU using Kalman filters. Much of this work is in the user
     * functions section at the end of this code. After using Kalman filtering to
     * improve the position estimate, relevant data is sent over serial to an ESP32
     * for data logging.
     **********************************************************************************/

    else if (state == 4){

      // buffers for holding normalized microphone buffers
      float32_t mic0_buf[ADC_BUF_LEN];
      float32_t mic1_buf[ADC_BUF_LEN];
      float32_t mic2_buf[ADC_BUF_LEN];
      float32_t mic3_buf[ADC_BUF_LEN];

      // de-interleave the ADC values and normalize the data
      int index = 0;
      for (int i = 0; i < ADC_BUF_LEN * NUM_MIC / 2; i += NUM_MIC / 2) {
        mic0_buf[index] = (((float)adc1conv(adc_buf[i]) - mic0_avg) / mic0_stdev);
        mic1_buf[index] = (((float)adc1conv(adc_buf[i+1]) - mic1_avg) / mic1_stdev);
        mic2_buf[index] = (((float)adc2conv(adc_buf[i]) - mic2_avg) / mic2_stdev);
        mic3_buf[index] = (((float)adc2conv(adc_buf[i+1]) - mic3_avg) / mic3_stdev);
        index += 1;
      }

      // calculate time-shifts between mic0 and all other microphones
      measured_time_shifts[0] = calculate_time_shift(mic0_buf, mic1_buf);
      measured_time_shifts[1] = calculate_time_shift(mic0_buf, mic2_buf);
      measured_time_shifts[2] = calculate_time_shift(mic0_buf, mic3_buf);

      // extract the current orientation of the platform
      // note: q0-3 are calculated in the Madgwick filter
      Quaternion quat_raw = {q0, q1, q2, q3};

      // create a quaternion to undo initial yaw rotation
      Quaternion yaw_compensation = create_yaw_quaternion(init_yaw);

      // apply the yaw compensation to the current quaternion
      Quaternion quat = multiply_quaternions(yaw_compensation, quat_raw);

      // normalize the result to ensure it's a valid rotation
      normalize_quaternion(&quat);

      // generate the euler angles associated with the compensated quaternion
      EulerAngles newAngles = QuaternionToEulerAngles(quat);

      // rotate the mic array by the compensated quaternion
      // note: this makes sure that the current orientation of the mic array is accounted
      // for before running Hooke-Jeeves search
      MicArray_t* rotated_mic_array = rotateMicArray(original_mic_array, quat);

      // the true position of the transmitter in meters
      // note: in the full (underwater) system, these would be the GPS coordinates of the
      // transmitter (encoded in the ultrasonic message) and the depth measured by a depth
      // pressure sensor. since I don't have those systems incorporated in this setup, I
      // measure the position physically and save it here.
      float true_x = 3.880f;
      float true_y = 0.0f;
      float true_z = -0.682f + 0.03;

      // define an initial guess for the transmitter's XYZ position
      // note: noise in the depth measurement is simulated here using randn()
      Point3D init_pos_est;
      init_pos_est.x = true_x - fossl_x + randn(0.0f, 0.005f);
      init_pos_est.y = 0;
      init_pos_est.z = 0;

      // perform hooke-jeeves search to minimize the residual function
      raw_isbl_pos_est = hooke_jeeves_search_2d(rotated_mic_array, init_pos_est, measured_time_shifts, 1000, 1e-13, 1e-6, 2.0f);

      // subtract estimated position to transmitter from known transmitter location to get receiver position
      // in global frame
      raw_isbl_pos_est.x = true_x - raw_isbl_pos_est.x;
      raw_isbl_pos_est.y = true_y - raw_isbl_pos_est.y;
      raw_isbl_pos_est.z = true_z - raw_isbl_pos_est.z;

      // calculate the change in time since the last acoustic update (deltat_pos)
      timestamp2 = __HAL_TIM_GET_COUNTER(&htim14);
      uint32_t diff_ticks;
      if (timestamp2 >= previousTimestamp2) {
          diff_ticks = timestamp2 - previousTimestamp2;
      } else {
          diff_ticks = (65536 - previousTimestamp2) + timestamp2;
      }
      deltat_pos = (float)diff_ticks * 65536.0f / 275000000.0f;
      previousTimestamp2 = __HAL_TIM_GET_COUNTER(&htim14);

      // update F and H matrices with new delta t
      update_kalman_matrices(kf_combined, kf_isbl, deltat_pos);

      // predict step for filters
      kalman_filter_predict(kf_combined);
      kalman_filter_predict(kf_isbl);

      // get measurements
      float32_t measurements_combined[6] = {
          raw_isbl_pos_est.x, raw_isbl_pos_est.y, raw_isbl_pos_est.z, delta_x_imu.x, delta_x_imu.y, delta_x_imu.z
      };
      float32_t measurements_isbl[3] = {
          raw_isbl_pos_est.x, raw_isbl_pos_est.y, raw_isbl_pos_est.z
      };

      // update step for filters
      kalman_filter_update(kf_combined, measurements_combined);
      kalman_filter_update(kf_isbl, measurements_isbl);

      // return filter position estimates, convert to mm
      kf_comb_pos_est.x = kf_combined->x_data[0] * 1000;
      kf_comb_pos_est.y = kf_combined->x_data[1] * 1000;
      kf_comb_pos_est.z = kf_combined->x_data[2] * 1000;

      kf_isbl_pos_est.x = kf_isbl->x_data[0] * 1000;
      kf_isbl_pos_est.y = kf_isbl->x_data[1] * 1000;
      kf_isbl_pos_est.z = kf_isbl->x_data[2] * 1000;

      // convert raw acoustic position estimate to mm
      raw_isbl_pos_est.x *= 1000;
      raw_isbl_pos_est.y *= 1000;
      raw_isbl_pos_est.z *= 1000;

      // convert true tx position to mm
      true_x *= 1000;
      true_y *= 1000;
      true_z *= 1000;

      // prepare and transmit the orientation string over UART
      char orientationString[120];
      snprintf(orientationString, sizeof(orientationString), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
               kf_isbl_pos_est.x, kf_isbl_pos_est.y, kf_isbl_pos_est.z,
               kf_comb_pos_est.x, kf_comb_pos_est.y, kf_comb_pos_est.z,
               raw_isbl_pos_est.x, raw_isbl_pos_est.y, raw_isbl_pos_est.z,
               newAngles.roll, newAngles.pitch, newAngles.yaw,
               true_x, true_y, true_z);
      HAL_UART_Transmit(&huart4, (uint8_t*)orientationString, strlen(orientationString), 200);

//				// send the full mic0-3 buffers over UART for debugging
//		    if (measured_time_shifts[1] > -0.0001){
//					// In your main code:
//					transmit_line(&huart4, "START_MIC_DATA\r");
//
//					char micDataString[40];
//					for (int i = 0; i < ADC_BUF_LEN; i++) {
//							snprintf(micDataString, sizeof(micDataString), "d: %.4f,%.4f,%.4f,%.4f\r",
//											 mic0_buf[i], mic1_buf[i], mic2_buf[i], mic3_buf[i]);
//							transmit_line(&huart4, micDataString);
//					}
//
//					transmit_line(&huart4, "END\r");
//		    }

      // get the next fossl_x value, the true x position of the FoSSL platform (will be removed when
      // transitioning to full system with depth sensor)
      float receivedValue;
      char confirmString[3];
      bad_depth_cnt = 0;
      HAL_StatusTypeDef rxStatus = receiveRobustFloat(&huart4, &receivedValue);
      if (rxStatus == HAL_OK) {
        fossl_x = receivedValue;
        snprintf(confirmString, sizeof(confirmString), "1");
        HAL_UART_Transmit(&huart4, (uint8_t*)confirmString, strlen(confirmString), 200);
      }

      // if first transmission doesn't work, then try again twice
      while ((rxStatus != HAL_OK) && (bad_depth_cnt < 3)){
        snprintf(confirmString, sizeof(confirmString), "0");
        HAL_UART_Transmit(&huart4, (uint8_t*)confirmString, strlen(confirmString), 200);
        rxStatus = receiveRobustFloat(&huart4, &receivedValue);
        if (rxStatus == HAL_OK) {
          fossl_x = receivedValue;
          snprintf(confirmString, sizeof(confirmString), "1");
          HAL_UART_Transmit(&huart4, (uint8_t*)confirmString, strlen(confirmString), 200);
        } else {
          bad_depth_cnt += 1;
        }
      }

      // if all three transmissions failed, let FoSSl know (sending 1 means good send, 0 means bad send,
      // -1 means all three failed and to stop trying). in this case, previous FoSSL value is used and the
      // next measurement is thrown out
      if (bad_depth_cnt >= 3){
        snprintf(confirmString, sizeof(confirmString), "-1");
        HAL_UART_Transmit(&huart4, (uint8_t*)confirmString, strlen(confirmString), 200);
      }

      // reset the position estimate
      raw_isbl_pos_est.x = 0;
      raw_isbl_pos_est.y = 0;
      raw_isbl_pos_est.z = 0;

      // reset the accumulated velocity values
      v_corrected.x = 0;
      v_corrected.y = 0;
      v_corrected.z = 0;
      delta_x_imu.x = 0;
      delta_x_imu.y = 0;
      delta_x_imu.z = 0;
      v_corrected_last_stable.x = 0;
      v_corrected_last_stable.y = 0;
      v_corrected_last_stable.z = 0;
      imu_i = 0;
      last_stable_i = 0;

      // reset the measured time shift array
      memset(measured_time_shifts, 0, sizeof(measured_time_shifts));

      // free the rotated mic array to prevent memory leak
      freeMicArray(rotated_mic_array);
      rotated_mic_array = NULL;  // Set to NULL to avoid using after free

      // reset all ADC buffers
      memset(adc_buf, 0, sizeof(adc_buf));
      memset(trig_buf, 0, sizeof(trig_buf));

      // turn off LEDs for debugging
      HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_RESET);  // Turn off green LED
      HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);  // Turn off yellow LED
      HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);  // Turn off red LED
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);  // Turn off LED on Pin E2

      // return to the trigger state
      state = 1;

      // restart the DMA from ADCs and the timer that controls the ADCs
      HAL_ADC_Start(&hadc2);
      HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)trig_buf, TRIG_BUF_LEN * NUM_MIC / 2);
      HAL_TIM_Base_Start_IT(&htim3);

      // add a delay to account for moving to the next position
      if (!stationary) HAL_Delay(2000);

    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 25;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;  /* ADC and DMA configured in resolution 32 bits to match with both ADC master and slave resolution */
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
//  hadc2.Init = hadc1.Init;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;

  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x00D049FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 323-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 4096-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65536-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 65536-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65536-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**********************************************************************************
 * ADC AND DMA USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Function that is called when the ADC has filled.
 * @param hadc The particular ADC that triggers the callback.
 *
 * This function has two purposes.
 *
 * First, if we are in the trigger state (state 1), we check if NUM_THRES_ROW
 * values in a row from a microphone exceed the TRIG_THRES value. If they do,
 * we set the flag exceedsThreshold to 1 and transition to the full recording
 * state. Otherwise, we remain in the trigger state and start the trigger data
 * collection again.
 *
 * Second, if we are in the full recording state (state 2), we transition to
 * the data processing state (state 3), since the adc_buf is full and we've
 * collected all our data.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

  // if we're in the trigger state, then trig_buf is full and needs to be tested
  if (state == 1){

  	// calculate mean for each microphone
  	float mic0_mean = 0, mic1_mean = 0, mic2_mean = 0, mic3_mean = 0;
  	for (int i = 0; i < TRIG_BUF_LEN * NUM_MIC/2; i += NUM_MIC/2) {
			mic0_mean += (float)adc1conv(trig_buf[i]);
			mic1_mean += (float)adc2conv(trig_buf[i]);
			mic2_mean += (float)adc1conv(trig_buf[i+1]);
			mic3_mean += (float)adc2conv(trig_buf[i+1]);
  	}
  	mic0_mean /= (float)TRIG_BUF_LEN;
  	mic1_mean /= (float)TRIG_BUF_LEN;
  	mic2_mean /= (float)TRIG_BUF_LEN;
  	mic3_mean /= (float)TRIG_BUF_LEN;

    // check if NUM_THRES_ROW values in a row from any microphone exceed TRIG_THRES
		int exceedsThresholdmic0 = 0;
		int exceedsThresholdmic1 = 0;
		int exceedsThresholdmic2 = 0;
		int exceedsThresholdmic3 = 0;
    int loopLen = TRIG_BUF_LEN - NUM_THRES_ROW;  /**< length of the for() loop for detecting if the trigger threshold has been exceeded */
    for (int i = 0; i < loopLen; i += 1) {
      int thres_count_mic0 = 0;
			int thres_count_mic1 = 0;
			int thres_count_mic2 = 0;
			int thres_count_mic3 = 0;

      // check if the next NUM_THRES_ROWS values for the current microphone exceed TRIG_THRES
      // remember that trig_buf stores interleaved values, so for the case of 4 mics and NUM_THRES_ROWS = 5,
      // the first loop should test the values trig_buf[0], trig_buf[4], trig_buf[8], ... trig_buf[16]
	    for (int t = 0; t < NUM_THRES_ROW; t++) {
				thres_count_mic0 += fabsf((float)adc1conv(trig_buf[i*NUM_MIC/2 + t*NUM_MIC/2 + 0]) - mic0_mean) > TRIG_THRES;
				thres_count_mic1 += fabsf((float)adc2conv(trig_buf[i*NUM_MIC/2 + t*NUM_MIC/2 + 0]) - mic1_mean) > TRIG_THRES;
				thres_count_mic2 += fabsf((float)adc1conv(trig_buf[i*NUM_MIC/2 + t*NUM_MIC/2 + 1]) - mic2_mean) > TRIG_THRES;
				thres_count_mic3 += fabsf((float)adc2conv(trig_buf[i*NUM_MIC/2 + t*NUM_MIC/2 + 1]) - mic3_mean) > TRIG_THRES;
	    }

      if (thres_count_mic0 == NUM_THRES_ROW) exceedsThresholdmic0 = 1;
      if (thres_count_mic1 == NUM_THRES_ROW) exceedsThresholdmic1 = 1;
      if (thres_count_mic2 == NUM_THRES_ROW) exceedsThresholdmic2 = 1;
      if (thres_count_mic3 == NUM_THRES_ROW) exceedsThresholdmic3 = 1;

      // If all mics have been checked, we can break the loop
			if (exceedsThresholdmic0 && exceedsThresholdmic1 && exceedsThresholdmic2 && exceedsThresholdmic3) {
					break;
			}
    }

    // mic0 controls the green LED
    if (exceedsThresholdmic0) {
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_SET);    // Turn on green LED
    } else {
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_RESET);  // Turn off green LED
    }

    // mic1 controls the yellow LED
    if (exceedsThresholdmic1) {
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);    // Turn on yellow LED
    } else {
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);  // Turn off yellow LED
    }

    // mic2 controls the red LED
    if (exceedsThresholdmic2) {
        HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);    // Turn on red LED
    } else {
        HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);  // Turn off red LED
    }

    // mic3 controls the LED on Pin E2
    if (exceedsThresholdmic3) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);     // Turn on LED on Pin E2
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   // Turn off LED on Pin E2
    }

    // Check how many microphones exceeded the threshold
    int exceedsThreshold = exceedsThresholdmic0 + exceedsThresholdmic1 + exceedsThresholdmic2 + exceedsThresholdmic3;

    // if at least two microphones exceeded the threshold, move to the full recording state
    if (exceedsThreshold > 1) {
        state = 2;                // transition to full recording state
        recording = 1;            // set recording flag to 1, system is now recording a long chunk and should not be interrupted
        HAL_ADC_Start(&hadc2);    // start the slave ADC

        // start the master ADC in DMA mode, write the ADC values to adc_buf
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN * NUM_MIC / 2);

    // otherwise, restart the triggering process
    } else {
        HAL_ADC_Start(&hadc2);    // start the slave ADC

        // start the master ADC in DMA mode, write the ADC values to trig_buf
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)trig_buf, TRIG_BUF_LEN * NUM_MIC / 2);
    }
  }

  // if we're in the full recording state, then adc_buf is full and we transition to the data processing state
  else if (state == 2){
    state = 3;
  }

}

/**
 * @brief Function that is called when DMA has finished transferring.
 * @param hdma The particular DMA instance that triggers the callback.
 *
 * Once the ADC has finished writing values to a buffer using DMA, this
 * function is called and the UART DMA mode is disabled to prevent future
 * transfers.
 */
void DMATransferComplete(DMA_HandleTypeDef *hdma) {

  // Disable UART DMA mode
  huart3.Instance->CR3 &= ~USART_CR3_DMAT;

}

/**********************************************************************************
 * TIME SHIFT CALCULATION USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Calculates the time shift between two time-series data using FFT cross-correlation.
 * @param micA_Buf The base time-series data buffer, usually mic0
 * @param micA_Buf The shifted time-series data buffer, usually mic1-3, whose time shift relative to mic0 is computed
 * @return shift Time shift between time-series data, in seconds
 *
 * This function calculates the time shift between two microphone recordings. It makes use of DSP functions from the
 * ARM libraries. The real-FFT instance must be initialized before this function is called.
 *
 * To speed up the computation, real-FFT convolution/correlation is used. This takes the computation time from O(n^2) time to
 * O(nlog(n)) time - for buffers of length 4096, it shortens the time by 4096/log(4096), or around 1134x. Details for
 * how this works can be found in my thesis text.
 *
 * The function follows this workflow:
 *    - Take micA buffer and add padding zeros to the end (necessary for convolution operation)
 *    - Calculate the real-FFT of the padded micA buffer
 *    - Take micB buffer and add padding zeros to the end
 *    - Calculate the real-FFT of the padded micB buffer
 *    - Calculate the complex conjugate of the real-FFT of the padded micB buffer
 *      (changes computation from convolution to cross-correlation)
 *    - Multiply the real-FFT of micA by the complex conjugate of real-FFT of micB
 *    - Calculate the inverse real-FFT of the product above
 *    - Find the index of the maximum positive value of the inverse real-FFT
 *      (finding absolute max value might give us values that are half a wavelength off)
 *    - Calculate the time shift based on the index of the max value
 *    - Return the time shift in seconds
 *
 * This function is called a total of (NUM_MIC - 1) times.
 */
float32_t calculate_time_shift(float32_t *micA_buf, float32_t *micB_buf) {
  // load buffers
  float32_t fftBufIn[FFT_BUF_LEN];    /**< stores input to FFT computation */
  float32_t fftBuf1Out[FFT_BUF_LEN];  /**< stores output of first FFT computation */
  float32_t sacrifice[FFT_BUF_LEN];   /**< buffer for handling memory leaks - something wrong with implementation and this fixes it */
  float32_t fftBuf2Out[FFT_BUF_LEN];  /**< stores output of second FFT computation */
  float32_t ifftBufIn[FFT_BUF_LEN];   /**< stores input to inverse FFT computation */
  float32_t ifftRealOut[FFT_BUF_LEN]; /**< stores output of inverse FFT computation */

  // reset buffers to 0
  memset(fftBufIn, 0, sizeof(fftBufIn));
  memset(fftBuf1Out, 0, sizeof(fftBuf1Out));
  memset(fftBuf2Out, 0, sizeof(fftBuf2Out));
  memset(ifftBufIn, 0, sizeof(ifftBufIn));
  memset(ifftRealOut, 0, sizeof(ifftRealOut));

  // load micA values into fftBufIn (remaining values zeros for zero-padding)
  memcpy(fftBufIn, micA_buf, ADC_BUF_LEN * sizeof(float32_t));

  // perform real fft on zero-padded micA values
  arm_rfft_fast_f32(&fftHandler, fftBufIn, fftBuf1Out, 0);

  // reset fftBufIn
  memset(fftBufIn, 0, sizeof(fftBufIn));

  // load micB values into fftBufIn (remaining values zeros for zero-padding)
  memcpy(fftBufIn, micB_buf, ADC_BUF_LEN * sizeof(float32_t));

  // perform real fft on zero-padded micB values, put result into temporary buf
  arm_rfft_fast_f32(&fftHandler, fftBufIn, ifftBufIn, 0);

  // calculate complex conjugate of micB real fft output (required for cross-correlation)
  arm_cmplx_conj_f32(ifftBufIn, fftBuf2Out, FFT_BUF_LEN);

  // reset temporary buf
  memset(ifftBufIn, 0, sizeof(ifftBufIn));

  // complex multiplication of the two fft output bufs
  arm_cmplx_mult_cmplx_f32(fftBuf1Out, fftBuf2Out, ifftBufIn, FFT_BUF_LEN);

  // perform inverse real fft on multiplication result
  arm_rfft_fast_f32(&fftHandler, ifftBufIn, ifftRealOut, 1);

  // find argmax((ifft_output)) to find time shift between signals
  int maxIndex = 0;
  float32_t maxVal = 0.0f;
  for (int i = 0; i < FFT_BUF_LEN; i++) {
    float32_t curVal = (ifftRealOut[i]);
    if (curVal > maxVal) {
      maxVal = curVal;
      maxIndex = i;
    }
  }

  // shift is maxIndex if maxIndex < 0.5*fft_len, otherwise is maxIndex - fft_len
  float32_t adj_index = (maxI ndex < FFT_BUF_LEN/2) ? (float)(maxIndex) : (float)(-FFT_BUF_LEN + maxIndex);

  // convert shift index to seconds (275MHz / 323 ADC clock)
  float32_t shift = adj_index / (275000000 / 323);

  // reset fft bufs
  memset(fftBufIn, 0, sizeof(fftBufIn));
  memset(fftBuf1Out, 0, sizeof(fftBuf1Out));
  memset(fftBuf2Out, 0, sizeof(fftBuf2Out));
  memset(ifftBufIn, 0, sizeof(ifftBufIn));
  memset(ifftRealOut, 0, sizeof(ifftRealOut));

  return shift;
}

/**********************************************************************************
 * MICROPHONE ARRAY CREATION AND MANIPULATION USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Creates a MicArray_t struct with a set number of mics
 * @param num_mics The number of mics in the mic_array
 * @return mic_array Pointer to MicArray_t struct
 *
 * This function creates a new MicArray_t struct with a specified number of mics. Note that it does not
 * assign coordinates to the mics; that is done with the function caclulate_base_points.
 */
MicArray_t* createMicArray(uint32_t num_mics) {
    MicArray_t* mic_array = (MicArray_t*)malloc(sizeof(MicArray_t) + num_mics * sizeof(Point3D));
    if (mic_array != NULL) {
        mic_array->num_mics = num_mics;
    }
    return mic_array;
}

/**
 * @brief Frees the memory associated with a MicArray_t struct
 * @param mic_array The pointer to the MicArray_t struct to be freed
 *
 * This function takes in a MicArray_t struct and frees the memory. This is essential to prevent memory
 * leaks when creating rotated mic arrays.
 */
void freeMicArray(MicArray_t* mic_array) {
    if (mic_array != NULL) {
        free(mic_array);
    }
}

/**
 * @brief Calculates the base points for a MicArray_t struct, assuming four mics and a square configuration
 * @param mic_array The pointer to the MicArray_t
 * @param baseline The spacing, in meters, between each microphone and the center of the array
 *
 * This function assigns 3D coordinates to each mic in a MicArray_t struct. Note that this function is
 * unique to the physical setup - different experimental setups require modifying this function.
 *
 * This implementation assumes the following configuration, with the +x axis coming out of the page:
 *
 *        3   0
 *   +y <---x
 *        2 | 1
 *          |
 *          v
 *         +z
 *
 * The baseline parameter is the distance from any microphone to the center (e.g. x -> 2 straight-line distance)
 */
void calculate_base_points(MicArray_t* mic_array, float baseline) {
    // convert the baseline to a YZ spacing factor
    const float factor = baseline / 1.414f;

    // set coordinates for each microphone
    mic_array->base_points[0] = (Point3D){0.0f, -factor, -factor};
    mic_array->base_points[1] = (Point3D){0.0f, -factor,  factor};
    mic_array->base_points[2] = (Point3D){0.0f,  factor,  factor};
    mic_array->base_points[3] = (Point3D){0.0f,  factor, -factor};

    // if there are more than 4 microphones, set their coordinates to (0,0,0)
    // note: this implementation only uses 4, so this is just for error catching
    for (uint32_t i = 4; i < mic_array->num_mics; i++) {
        mic_array->base_points[i] = (Point3D){0.0f, 0.0f, 0.0f};
    }
}

/**
 * @brief Rotates a MicArray_t struct by a Quaternion struct
 * @param original_mic_array The pointer to the MicArray_t struct to be rotated
 * @param quat The Quaternion struct to rotate the MicArray_t with
 * @return rotated_mic_array The pointer to the rotated MicArray_t struct
 *
 * This function is used to rotate MicArray_t structs by a quaternion. It rotates each microphone's 3D position
 * by the quaternion (assuming the quaternion rotates about the center of the array). To do this, it first creates
 * a new MicArray_t struct, then calculates where each mic would move to when the rotation is applied to the original
 * positions.
 *
 * It is imperative that the freeMicArray() function is called after the rotated mic array has been used in the main
 * loop. Otherwise, each time this function gets called, it will create a new MicArray_t struct and memory will run out.
 * It may be desired to change this function and take in an additional MicArray_t as a parameter, so that the process
 * is more clear to the user.
 */
MicArray_t* rotateMicArray(MicArray_t* original_mic_array, Quaternion quat) {
    if (original_mic_array == NULL) {
        return NULL;
    }

    MicArray_t* rotated_mic_array = createMicArray(original_mic_array->num_mics);
    if (rotated_mic_array == NULL) {
        return NULL;
    }

    for (uint32_t i = 0; i < original_mic_array->num_mics; i++) {
        rotated_mic_array->base_points[i] = rotatePoint(original_mic_array->base_points[i], quat);
    }

    return rotated_mic_array;
}


/**********************************************************************************
 * HOOKE-JEEVES SEARCH USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Performs Hooke-Jeeves Search residual minimization locked to a 2D plane (initial X estimate)
 * @param mic_array The pointer to the MicArray_t struct that defines the 3D position of each microphone
 * @param init_pos_est The initial position estimate of the transmitter
 * @param measured_time_shifts The measured time shifts between microphones, as calculated by calculate_time_shifts function
 * @param max_iter If this number of iterations are exceeded, the loop is exited
 * @param min_residual If the residual goes below this number, the loop is exited
 * @param min_spacing If the spacing (distance between steps) goes below this number, the loop is exited
 * @param scale_factor How much the spacing should be reduced by each time the direction changes (spacing /= scale_factor)
 * @return pos_est The best position estimate of the transmitter after running Hooke-Jeeves Search
 *
 * This function is a 2D implementation of Hooke-Jeeves Search (HJS), an algorithm that finds a point in n-dimensional space that
 * minimizes a residual (or cost) function. HJS follows this procedure:
 *    - Start with step size S and initial position estimate X[1, 2, ... n]
 *    - Find the residual for the initial positon estiamte
 *    - For each dimension i, test the points X[i] - S and X[i] + S
 *    - Find which direction of movement gives the smallest residual and move in that direction
 *    - Continue moving in that direction until residual begins to increase, then divide step size by scale_factor
 *    - Start step 3 with new step size, repeat until an exit condition has been met
 *
 * The exit conditions are:
 *    - The number of iterations exceeds max_iter
 *    - The residual returned is less than min_residual
 *    - The spacing is set to less than min_spacing
 *
 * More on the theory of HJS can be found in my thesis text.
 */
Point3D hooke_jeeves_search_2d(MicArray_t* mic_array, Point3D init_pos_est, float32_t measured_time_shifts[], int max_iter, float32_t min_residual, float32_t min_spacing, float32_t scale_factor) {
	Point3D pos_est = init_pos_est;       /**< best position estimate */
	Point3D new_pos_est = init_pos_est;   /**< new position estimate for this iteration */
  float32_t residual = 1e9;             /**< residual for current position estimate */
  float32_t residual_arr[4];            /**< list of residuals for testing new directions */
  float32_t prev_residual = 1e10;       /**< residual for previous position estiamte */
  float32_t spacing = 2;                /**< step size for new position estimates in best direction, in meters */
  int new_dir_index = 0;                /**< index of direction with lowest residual */
  int new_dir_flag = 0;                 /**< flag sets to 1 if new direction should be chosen */
  int iter = 0;                         /**< current iteration of HJS */
  int dy = 0;                           /**< direction modifier for y-axis */
  int dz = 0;                           /**< direction modifier for z-axis */

  while ((iter < max_iter) && (residual > min_residual) && (spacing > min_spacing)){

    // if first iteration or previous movement gave a worse residual, choose a new direction
    if (new_dir_flag){

      // every time a new direction is chosen, spacing is reduced by 1/scale_factor
      // set scale_factor between (1,2]
      spacing /= scale_factor;

      // test plus/minus spacing in each dimension and save the residuals
      int d_idx = 0;
      for (int dy_ = -1; dy_ <= 1; dy_ += 2){
        new_pos_est.y = pos_est.y + (dy_) * spacing;
        new_pos_est.z = pos_est.z;
        residual_arr[d_idx] = calculate_residual(mic_array, new_pos_est, measured_time_shifts);
        d_idx ++;
      }
      for (int dz_ = -1; dz_ <= 1; dz_ += 2){
        new_pos_est.y = pos_est.y;
        new_pos_est.z = pos_est.z + (dz_) * spacing;
        residual_arr[d_idx] = calculate_residual(mic_array, new_pos_est, measured_time_shifts);
        d_idx ++;
      }

      // find direction that gives smallest residual
      new_dir_index = arg_min(residual_arr, d_idx);

      // if that direction has a smaller residual than the previous residual, move in that direction
      if (residual_arr[new_dir_index] < residual){
        residual = residual_arr[new_dir_index];

        // determine what the new direction is
        if (new_dir_index < 2) {
          dy = (new_dir_index % 2) * 2 - 1;
          dz = 0;
        } else {
          dy = 0;
          dz = ((new_dir_index - 2) % 2) * 2 - 1;
        }

        // calculate new position based on that direction
        new_pos_est.y = pos_est.y + (dy) * spacing;
        new_pos_est.z = pos_est.z + (dz) * spacing;

        // save as new best position estimate
        pos_est = new_pos_est;

        // start moving in that direction until improvement stops
        new_dir_flag = 0;
      }

      // otherwise, scale down again
      else{
        new_dir_flag = 1;
      }
    }

    // otherwise, continue in that direction
    else{
      // calculate residual from moving in that direction with current spacing
      prev_residual = residual;
      new_pos_est.y = pos_est.y + (dy) * spacing;
      new_pos_est.z = pos_est.z + (dz) * spacing;
      residual = calculate_residual(mic_array, new_pos_est, measured_time_shifts);

      // if new residual is smaller than previous residual, keep moving in that direction
      if (residual < prev_residual){
        pos_est = new_pos_est;
      }

      // otherwise, choose a new direction next iteration
      else{
        residual = prev_residual
        new_dir_flag = 1;
      }
    }

    iter++;
  }

  // once an exit condition has been satisfied, break the loop and return the current position estimate
  return pos_est;
}

/**
 * @brief Performs Hooke-Jeeves Search residual minimization in 3D
 * @param mic_array The pointer to the MicArray_t struct that defines the 3D position of each microphone
 * @param init_pos_est The initial position estimate of the transmitter
 * @param measured_time_shifts The measured time shifts between microphones, as calculated by calculate_time_shifts function
 * @param max_iter If this number of iterations are exceeded, the loop is exited
 * @param min_residual If the residual goes below this number, the loop is exited
 * @param min_spacing If the spacing (distance between steps) goes below this number, the loop is exited
 * @param scale_factor How much the spacing should be reduced by each time the direction changes (spacing /= scale_factor)
 * @return pos_est The best position estimate of the transmitter after running Hooke-Jeeves Search
 *
 * This function is a 3D implementation of Hooke-Jeeves Search (HJS), an algorithm that finds a point in n-dimensional space that
 * minimizes a residual (or cost) function. HJS follows this procedure:
 *    - Start with step size S and initial position estimate X[1, 2, ... n]
 *    - Find the residual for the initial positon estiamte
 *    - For each dimension i, test the points X[i] - S and X[i] + S
 *    - Find which direction of movement gives the smallest residual and move in that direction
 *    - Continue moving in that direction until residual begins to increase, then divide step size by scale_factor
 *    - Start step 3 with new step size, repeat until an exit condition has been met
 *
 * The exit conditions are:
 *    - The number of iterations exceeds max_iter
 *    - The residual returned is less than min_residual
 *    - The spacing is set to less than min_spacing
 *
 * More on the theory of HJS can be found in my thesis text.
 */
Point3D hooke_jeeves_search_3d(MicArray_t* mic_array, Point3D init_pos_est, float32_t measured_time_shifts[], int max_iter, float32_t min_residual, float32_t min_spacing, float32_t scale_factor) {
  Point3D pos_est = init_pos_est;       /**< best position estimate */
  Point3D new_pos_est = init_pos_est;   /**< new position estimate for this iteration */
  float32_t residual = 1e9;             /**< residual for current position estimate */
  float32_t residual_arr[4];            /**< list of residuals for testing new directions */
  float32_t prev_residual = 1e10;       /**< residual for previous position estiamte */
  float32_t spacing = 2;                /**< step size for new position estimates in best direction, in meters */
  int new_dir_index = 0;                /**< index of direction with lowest residual */
  int new_dir_flag = 0;                 /**< flag sets to 1 if new direction should be chosen */
  int iter = 0;                         /**< current iteration of HJS */
  int dx = 0;                           /**< direction modifier for x-axis */
  int dy = 0;                           /**< direction modifier for y-axis */
  int dz = 0;                           /**< direction modifier for z-axis */

  while ((iter < max_iter) && (residual > min_residual) && (spacing > min_spacing)){

    // if first iteration or previous movement gave a worse residual, choose a new direction
    if (new_dir_flag){

      // every time a new direction is chosen, spacing is reduced by 1/scale_factor
      // set scale_factor between (1,2]
      spacing /= scale_factor;

      // test plus/minus spacing in each dimension and save the residuals
      int d_idx = 0;
      for (int dx_ = -1; dx_ <= 1; dx_ += 2){
        new_pos_est.x = pos_est.x + (dx_) * spacing;
        new_pos_est.y = pos_est.y;
        new_pos_est.z = pos_est.z;
        residual_arr[d_idx] = calculate_residual(mic_array, new_pos_est, measured_time_shifts);
        d_idx ++;
      }
      for (int dy_ = -1; dy_ <= 1; dy_ += 2){
        new_pos_est.x = pos_est.x;
        new_pos_est.y = pos_est.y + (dy_) * spacing;
        new_pos_est.z = pos_est.z;
        residual_arr[d_idx] = calculate_residual(mic_array, new_pos_est, measured_time_shifts);
        d_idx ++;
      }
      for (int dz_ = -1; dz_ <= 1; dz_ += 2){
        new_pos_est.x = pos_est.x;
        new_pos_est.y = pos_est.y;
        new_pos_est.z = pos_est.z + (dz_) * spacing;
        residual_arr[d_idx] = calculate_residual(mic_array, new_pos_est, measured_time_shifts);
        d_idx ++;
      }

      // find direction that gives smallest residual
      new_dir_index = arg_min(residual_arr, d_idx);

      // if that direction has a smaller residual than the previous residual, move in that direction
      if (residual_arr[new_dir_index] < residual){
        residual = residual_arr[new_dir_index];

        // determine what the new direction is
        if (new_dir_index < 2) {
          dx = (new_dir_index % 2) * 2 - 1;
          dy = 0;
          dz = 0;
        } else if (new_dir_index < 4) {
          dx = 0;
          dy = ((new_dir_index - 2) % 2) * 2 - 1;
          dz = 0;
        } else {
          dx = 0;
          dy = 0;
          dz = ((new_dir_index - 4) % 2) * 2 - 1;
        }

        // calculate new position based on that direction
        new_pos_est.x = pos_est.x + (dx) * spacing;
        new_pos_est.y = pos_est.y + (dy) * spacing;
        new_pos_est.z = pos_est.z + (dz) * spacing;

        // save as new best position estimate
        pos_est = new_pos_est;

        // start moving in that direction until improvement stops
        new_dir_flag = 0;
      }

      // otherwise, scale down again
      else{
        new_dir_flag = 1;
      }
    }

    // otherwise, continue in that direction
    else{
      // calculate residual from moving in that direction with current spacing
      prev_residual = residual;
      new_pos_est.x = pos_est.x + (dx) * spacing;
      new_pos_est.y = pos_est.y + (dy) * spacing;
      new_pos_est.z = pos_est.z + (dz) * spacing;
      residual = calculate_residual(mic_array, new_pos_est, measured_time_shifts);

      // if new residual is smaller than previous residual, keep moving in that direction
      if (residual < prev_residual){
        pos_est = new_pos_est;
      }

      // otherwise, choose a new direction next iteration
      else{
        residual = prev_residual
        new_dir_flag = 1;
      }
    }

    iter++;
  }

  // once an exit condition has been satisfied, break the loop and return the current position estimate
  return pos_est;
}

/**
 * @brief Calculates the residual for a given target point, measured time shifts, and receiver array
 * @param mic_array The pointer to the MicArray_t struct with the location of each microphone
 * @param target_point The 3D position estimate of the transmitter being tested
 * @param measured_time_shifts The time shifts between each microphones' recordings
 * @return residual The squared difference between the measured time shifts and the time shifts for the tested target point
 *
 * This function is the residual (or cost) function being minimized in the Hooke-Jeeves Search (HJS). It takes in a MicArray_t
 * struct (which compensates for the orientation of the array), a 3D test point for the transmitter, and the measured time
 * shifts between the microphones, and returns a single value: the squared difference between the measured time shifts and
 * the time shifts that would be expected if the transmitter was at the target point.
 *
 * Essentially, this function takes the target point and mic array positions, and computes what time shifts would be seen if
 * the transmitter was truly at that test point. The residual is the sum of squared differences of the measured and calculated
 * time shifts (calculated refers to being calculated for that particular test point).
 */
float32_t calculate_residual(MicArray_t* mic_array, Point3D target_point, float32_t measured_time_shifts[]) {
  float32_t toa0 = 0;   /**< time of arrival for first mic */
  float32_t toaN = 0;   /**< time of arrival for Nth mic */
  float32_t* target_time_shifts = malloc((mic_array->num_mics - 1) * sizeof(float32_t));

  toa0 = calculate_toa(mic_array->base_points[0], target_point, sound_speed_mps);

  // for each mic, calculate the time of arrival (distance from target point to that mic times speed of sound)
  // for underwater implementation, sound speed is not constant and depends on depth, temp, and salinity - this would need to
  // be modified to account for those factors
  for (int i = 1; i < mic_array->num_mics; i++) {
    toaN = calculate_toa(mic_array->base_points[i], target_point, sound_speed_mps);

    // the time shift is the time of arrival to the first mic minus the time of arrival to the Nth mic
    target_time_shifts[i-1] = toa0 - toaN;
  }

  // calculate the residual by taking the sum of squared differences between the time shift arrays
  float32_t residual = calculate_squared_diff(measured_time_shifts, target_time_shifts, mic_array->num_mics - 1);

  // save the calculated time shifts into a global array (for debugging / comparing to the real estimates)
  // future work: implement the residual for the final estimate into the Kalman filter to increase uncertainty?
  for (int i = 0; i < mic_array->num_mics - 1; i++) {
    calcd_time_shifts[i] = target_time_shifts[i];
  }

  // make sure to free the memory for the target time shift array to avoid memory leak
  free(target_time_shifts);
  return residual;
}

/**
 * @brief Calculates the sum of squared differences between two lists
 * @param measured_data The first list, usually measured data
 * @param target_data The second list, usually target/calculated data
 * @param data_len The number of entries in each list
 * @return squared_diff The sum of squared differences between the two lists
 */
float32_t calculate_squared_diff(float32_t measured_data[], float32_t target_data[], int data_len){
  float32_t squared_diff = 0;
  for (int i = 0; i < data_len; i++){
    squared_diff += ((measured_data[i] - target_data[i])*(measured_data[i] - target_data[i]));
  }
  return squared_diff;
}

/**
 * @brief Calculates the time of arrival between two 3D points given a propogation speed
 * @param pointA The 3D coordinates of the first point
 * @param pointB The 3D coordinates of the second point
 * @param sound_speed_mps The speed of sound in meters per second
 * @return toa Time of arrival between the two points
 *
 * This function assume a constant speed of sound, which is not valid for large changes in depth in water.
 */
float32_t calculate_toa(Point3D pointA, Point3D pointB, float32_t sound_speed_mps) {
  float32_t xA = pointA.x;
  float32_t yA = pointA.y;
  float32_t zA = pointA.z;
  float32_t xB = pointB.x;
  float32_t yB = pointB.y;
  float32_t zB = pointB.z;
  float32_t euclidean_dist = sqrt((xA-xB)*(xA-xB) + (yA-yB)*(yA-yB) + (zA-zB)*(zA-zB));
  float32_t toa = euclidean_dist / sound_speed_mps;
  return toa;
}

/**
 * @brief Finds the index of the minimum value of a list
 * @param data_array The data list to be searched
 * @param dat_len The length of the data list
 * @return minIndex The index of the minimum value of the list
 */
int arg_min(float32_t data_array[], int data_len){
  float32_t minValue = 1e10;
  int minIndex = 0;
  for (int i = 0; i < data_len; i++) {
    if (data_array[i] < minValue) {
      minValue = data_array[i];
      minIndex = i;
    }
  }
  return minIndex;
}

/**********************************************************************************
 * IMU READING/WRITING USER FUNCTIONS
 **********************************************************************************/

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 * Written by STMicroelectronics, from documentation here:
 * @see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm6dsox_STdC
 */
static int32_t lsm6dsox_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 * Written by STMicroelectronics, from documentation here:
 * @see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm6dsox_STdC
 */
static int32_t lsm6dsox_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/**
 * @brief Handler function called when new data is ready to be processed from LSM6DSOX
 *
 * This function is called when the accelerometer and gyroscope data is ready to read.
 *
 * Note that a coordinate transform is applied here; the sensor frame of the IMU relative to the global frame
 * of the iSBL array is as shown below.
 *    - Positive X-axis of IMU points between the negative Y and positive Z axes of the iSBL array
 *    - Positive Y-axis of IMU points between the negative Y and negative Z axes of the iSBL array
 *    - Positive Z-axis of IMU points towards the positive X axis of the iSBL array
 */
void lsm6dsox_read_data_drdy_handler(void)
{
	uint8_t reg;

	/* Read output only if new xl value is available */
	lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
	    /* Read acceleration field data */
	    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	    lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration);                               // Note: acceleration data is negative by default
	    acceleration_g[0] = -lsm6dsox_from_fs4_to_mg(data_raw_acceleration[2]) / 1000.0;              // Z-axis reading to X
	    acceleration_g[1] = 0.7071 * (lsm6dsox_from_fs4_to_mg(data_raw_acceleration[0]) / 1000.0 +
	                                  lsm6dsox_from_fs4_to_mg(data_raw_acceleration[1]) / 1000.0);    // Combination of X and Y to Y
	    acceleration_g[2] = 0.7071 * (-lsm6dsox_from_fs4_to_mg(data_raw_acceleration[0]) / 1000.0 -
	                                  -lsm6dsox_from_fs4_to_mg(data_raw_acceleration[1]) / 1000.0);   // Combination of X and Y to Z
	}

	lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
	    /* Read angular rate field data */
	    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	    lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
	    angular_rate_dps[0] = lsm6dsox_from_fs1000_to_mdps(data_raw_angular_rate[2]) / 1000.0;              // Z-axis reading to X
	    angular_rate_dps[1] = 0.7071 * (-lsm6dsox_from_fs1000_to_mdps(data_raw_angular_rate[0]) / 1000.0 +
	                                    -lsm6dsox_from_fs1000_to_mdps(data_raw_angular_rate[1]) / 1000.0);  // Combination of X and Y to Y
	    angular_rate_dps[2] = 0.7071 * (lsm6dsox_from_fs1000_to_mdps(data_raw_angular_rate[0]) / 1000.0 -
	                                    lsm6dsox_from_fs1000_to_mdps(data_raw_angular_rate[1]) / 1000.0);   // Combination of X and Y to Z

	    if (imu_calibrated){

				// Apply gyroscope calibration only if it has already been calibrated
				angular_rate_dps[0] += g_cal[0];
				angular_rate_dps[1] += g_cal[1];
				angular_rate_dps[2] += g_cal[2];
	    }
	}

	lsm6dsox_temp_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
		/* Read temperature data */
		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		lsm6dsox_temperature_raw_get(&dev_ctx, &data_raw_temperature);
		temperature_degC = lsm6dsox_from_lsb_to_celsius(
												 data_raw_temperature);
	}
}

/**
 * @brief Initialization routine for LSM6DSOX
 *
 * Written by STMicroelectronics, from documentation here:
 * @see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm6dsox_STdC
 *
 * Set data rates and sensitivities here.
 */
void lsm6dsox_read_data_drdy(void)
{
  lsm6dsox_pin_int2_route_t int2_route;
  lsm6dsox_status_reg_t status;
  uint8_t reg;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = lsm6dsox_write;
  dev_ctx.read_reg = lsm6dsox_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(100);

  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID){
      while (1){
          NVIC_SystemReset();
      }
  }

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
      lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);

//  /* Enable Block Data Update */
//  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  lsm6dsox_pin_int2_route_get(&dev_ctx, NULL, &int2_route);
//  int2_route.drdy_xl = PROPERTY_ENABLE;
  int2_route.drdy_g = PROPERTY_ENABLE;  // Enable gyro data ready interrupt
  lsm6dsox_pin_int2_route_set(&dev_ctx, NULL, int2_route);

  /* Set Output Data Rate */
  lsm6dsox_xl_power_mode_set(&dev_ctx, LSM6DSOX_HIGH_PERFORMANCE_MD);
  lsm6dsox_gy_power_mode_set(&dev_ctx, LSM6DSOX_GY_HIGH_PERFORMANCE);
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_208Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_208Hz);

  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_4g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_1000dps);

  // Disable FIFO
	lsm6dsox_fifo_mode_set(&dev_ctx, LSM6DSOX_BYPASS_MODE);

  // Set interrupt to pulsed mode
  lsm6dsox_int_notification_set(&dev_ctx, LSM6DSOX_ALL_INT_PULSED);

  /* Check if gyro is enabled */
  lsm6dsox_read_reg(&dev_ctx, LSM6DSOX_CTRL2_G, &reg, 1);
  if ((reg & 0xF0) == 0) {
      printf("Gyro is not enabled!\n");
  }

  /* Wait for a moment */
  platform_delay(3000);

  /* Check status */
  lsm6dsox_status_reg_get(&dev_ctx, &status);
  printf("Status: XL_DA: %d, GY_DA: %d\n", status.xlda, status.gda);

  imu_rdy = 1;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 * Written by STMicroelectronics, from documentation here:
 * @see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis3mdl_STdC
 */
static int32_t lis3mdl_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 * Written by STMicroelectronics, from documentation here:
 * @see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis3mdl_STdC
 */
static int32_t lis3mdl_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/**
 * @brief Handler function called when new data is ready to be processed from LIS3MDL
 *
 * This function is called when the magnetometer data is ready to read.
 *
 * Note that a coordinate transform is applied here; the sensor frame of the IMU relative to the global frame
 * of the iSBL array is as shown below.
 *    - Positive X-axis of IMU points between the negative Y and positive Z axes of the iSBL array
 *    - Positive Y-axis of IMU points between the negative Y and negative Z axes of the iSBL array
 *    - Positive Z-axis of IMU points towards the positive X axis of the iSBL array
 */
void lis3mdl_read_data_drdy_handler(void)
{
	uint8_t reg;
//	char *exceeds_info;
//	lis3mdl_int_src_t source;
	/* Read output only if new value is available
	 * It's also possible to use interrupt pin for trigger
	 */
	lis3mdl_mag_data_ready_get(&dev_ctx_m, &reg);

	if (reg) {
	    /* Read magnetic field data */
	    memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
	    lis3mdl_magnetic_raw_get(&dev_ctx_m, data_raw_magnetic);
	    magnetic_G[0] = lis3mdl_from_fs4_to_gauss(data_raw_magnetic[2]);              // Z-axis reading to X
	    magnetic_G[1] = 0.7071 * (-lis3mdl_from_fs4_to_gauss(data_raw_magnetic[0]) +
	                              -lis3mdl_from_fs4_to_gauss(data_raw_magnetic[1]));  // Combination of X and Y to Y
	    magnetic_G[2] = 0.7071 * (lis3mdl_from_fs4_to_gauss(data_raw_magnetic[0]) -
	                              lis3mdl_from_fs4_to_gauss(data_raw_magnetic[1]));   // Combination of X and Y to Z
	}
}

/**
 * @brief Initialization routine for LIS3MDL
 *
 * Written by STMicroelectronics, from documentation here:
 * @see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis3mdl_STdC
 *
 * Set data rate and sensitivity here.
 */
void lis3mdl_read_data_drdy(void)
{
  lis3mdl_int_cfg_t int_ctrl;
  dev_ctx_m.write_reg = lis3mdl_write;
  dev_ctx_m.read_reg = lis3mdl_read;
  dev_ctx_m.mdelay = platform_delay;
  dev_ctx_m.handle = &SENSOR_BUS;
  /* Wait sensor boot time */
  platform_delay(100);
  /* Check device ID */
  lis3mdl_device_id_get(&dev_ctx_m, &whoamI);

  if (whoamI != LIS3MDL_ID) {
    while (1) {
      /* manage here device not found */
    	NVIC_SystemReset();
    }
  }

  /* Restore default configuration */
  lis3mdl_reset_set(&dev_ctx_m, PROPERTY_ENABLE);

  do {
    lis3mdl_reset_get(&dev_ctx_m, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lis3mdl_block_data_update_set(&dev_ctx_m, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lis3mdl_data_rate_set(&dev_ctx_m, LIS3MDL_HP_80Hz);
  /* Set full scale */
  lis3mdl_full_scale_set(&dev_ctx_m, LIS3MDL_4_GAUSS);
  /* Enable temperature sensor */
  lis3mdl_temperature_meas_set(&dev_ctx_m, PROPERTY_ENABLE);
  /* Set device in continuous mode */
  lis3mdl_operating_mode_set(&dev_ctx_m, LIS3MDL_CONTINUOUS_MODE);
  /* Enable interrupt generation on interrupt */
  lis3mdl_int_generation_set(&dev_ctx_m, PROPERTY_ENABLE);
  /* Set interrupt threshold
   *
   * The sample code exploits a threshold and notify it by
   * hardware through the INT/DRDY pin
   */
  lis3mdl_int_threshold_set(&dev_ctx_m, mag_threshold);
  int_ctrl.iea = PROPERTY_DISABLE;
  int_ctrl.ien = PROPERTY_DISABLE;
  int_ctrl.lir = PROPERTY_DISABLE;
  int_ctrl.zien = PROPERTY_DISABLE;
  int_ctrl.yien = PROPERTY_DISABLE;
  int_ctrl.xien = PROPERTY_DISABLE;
  lis3mdl_int_config_set(&dev_ctx_m, &int_ctrl);
  HAL_Delay(1);
}

/**
 * @brief Delay function for this platform (STM32)
 * @param ms Time delay in ms
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/**********************************************************************************
 * MADGWICK FILTERING AND IMU INITIALIZATION USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Calibrates the gyroscope (system must be stationary when called!)
 * @param g_cal X, Y, and Z calibration constants for gyroscope (will be filled when program is run)
 * @param num_item Number of readings to take (and average over)
 */
void gyro_calibration(float g_cal[3], int num_iter){
	for (int i = 0; i < num_iter; i++){
		while (!data_rdy);
		g_cal[0] -= angular_rate_dps[0];
		g_cal[1] -= angular_rate_dps[1];
		g_cal[2] -= angular_rate_dps[2];
		data_rdy = 0;
	}
	g_cal[0] /= (float)num_iter;
	g_cal[1] /= (float)num_iter;
	g_cal[2] /= (float)num_iter;
	imu_calibrated = 1;
}

/**
 * @brief Initializes the IMU (LIS3MDL and LSM6DSOX) and runs gyroscope calibration (system must be stationary!)
 *
 * This function runs the initialization routines for the LIS3MDL magnetometer and LSM6DSOX accelerometer and
 * gyroscope. Then, it runs the gyroscope calibration routine over 256 readings to remove gyroscope biases
 * from future readings.
 */
void init_imu(void){
  lis3mdl_read_data_drdy();
  lsm6dsox_read_data_drdy();
  lis3mdl_read_data_drdy_handler();
  lsm6dsox_read_data_drdy_handler();
  gyro_calibration(g_cal, 256);
}

/**
 * @brief Runs an update for the Madgwick filter
 *
 * The first part of this function is an implementation of the Madgwick filter. It gets called once the accelerometer and
 * gyroscope update with new data and runs through one iteration. It assumes the raw data is in the correct coordinate system.
 * It stores the current orientation estimate of the system using quaternions.
 *
 * The second part of this function implements dead-reckoning for position estimation. First, it waits until 10000 filter
 * updates have run, and then saves the quaternion as the "initial quaternion" - for testing, this was used for a custom
 * coordinate system where the initial orientation of the IMU was considered "zeroed out." In the full implementation, this
 * would not be used.
 *
 * Next, if the filter has initialized, it converts the sensor-frame accelerometer data into the global frame and performs
 * double integration to get a position estimate. See the function code for more details.
 *
 * See my thesis text and the original Madgwick paper for the theory implemented into this function.
 *
 * @see https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */
void update_filter(void){
	// calculate the time taken since the last update
	timestamp = __HAL_TIM_GET_COUNTER(&htim13);
	uint32_t diff_ticks;
	if (timestamp >= previousTimestamp) {
		diff_ticks = timestamp - previousTimestamp;
	} else {
		diff_ticks = (65536 - previousTimestamp) + timestamp;
	}
	deltat = (float)diff_ticks * 4096.0f / 275000.0f;
	previousTimestamp = __HAL_TIM_GET_COUNTER(&htim13);

	// pre-allocate values for speeding up computation
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// get raw data
	float ax = acceleration_g[0];
	float ay = acceleration_g[1];
	float az = acceleration_g[2];
	float gx = angular_rate_dps[0] * 3.14159 / 180.0;
	float gy = angular_rate_dps[1] * 3.14159 / 180.0;
	float gz = angular_rate_dps[2] * 3.14159 / 180.0;
	float mx = magnetic_G[0];
	float my = magnetic_G[1];
	float mz = magnetic_G[2];

  // rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// normalise accelerometer measurement
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// normalise magnetometer measurement
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;

	// auxiliary variables to avoid repeated arithmetic
	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q0q2 = 2.0f * q0 * q2;
	_2q2q3 = 2.0f * q2 * q3;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// reference direction of Earth's magnetic field
	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// gradient decent algorithm corrective step
	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

	// normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// apply feedback step
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;

	// integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat / 1000;
	q1 += qDot2 * deltat / 1000;
	q2 += qDot3 * deltat / 1000;
	q3 += qDot4 * deltat / 1000;

	// normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// if the filter has converged, run the dead-reckoning procedure
	if (imu_i >= 0) {
	  // find the magnitude of the acceleration readings and subtract the gravity vector
		float acc_mag_abs = fabsf(sqrtf(acceleration_g[0]*acceleration_g[0] + acceleration_g[1]*acceleration_g[1] + acceleration_g[2]*acceleration_g[2]) - 1);

    // extract the current orientation of the platform
    Quaternion quat_raw = {q0, q1, q2, q3};

    // create a quaternion to undo initial yaw rotation and apply it to the current quaternion
    Quaternion yaw_compensation = create_yaw_quaternion(init_yaw);
    Quaternion quat = multiply_quaternions(yaw_compensation, quat_raw);

    // normalize the result to ensure it's a valid rotation
    normalize_quaternion(&quat);

    // rotate the accelerometer readings by the current quaternion to get the acceleration from the sensor frame to the global frame
		Point3D acc_temp = {acceleration_g[0], acceleration_g[1], acceleration_g[2]};
		Point3D acc_corr_temp = rotatePoint(acc_temp, quat);

		// if the acceleration magnitude is greater than a threshold, then integrate the acceleration into a change in velocity
		if (acc_mag_abs > 0.04){
			v_corrected.x -= (acc_corr_temp.x) * deltat/1000 * 9.81;    // acceleration is in g's, convert to m/s^2
			v_corrected.y -= (acc_corr_temp.y) * deltat/1000 * 9.81;
			v_corrected.z -= (acc_corr_temp.z - 1) * deltat/1000 * 9.81;
			acc_stable_cnt = 0;
			acc_stable_flag = 1;
		}

		// otherwise, the IMU is stable
		else{
			acc_stable_cnt++;

			// if the IMU has been stable for a set number of filter updates, then consider it to be stationary and account for velocity drift
			if ((acc_stable_cnt >= 20) && acc_stable_flag){

			  // find how many filter updates have passed since the last time the IMU was last stable
				float i_diff = (imu_i > last_stable_i) ? (float) imu_i - last_stable_i : (float) 10000 - last_stable_i + imu_i;
				i_diff += 1;
				i_diff -= (float) acc_stable_cnt;

				// if the system has integrated acceleration properly, and assuming that "stationary" (aka, the accelerometer has not registered any
				// significant accelerations) means that the velocity is zero, then account for the drift in velocity over time
				Point3D v_int_drift;
				v_int_drift.x = v_corrected.x - v_corrected_last_stable.x;  // the current velocity should be the same as the last stable velocity,
				v_int_drift.y = v_corrected.y - v_corrected_last_stable.y;  // assuming that the integration was without errors; this drift term
				v_int_drift.z = v_corrected.z - v_corrected_last_stable.z;  // accounts for the velocity drift since then

				// in order to remove the velocity drift from the delta_x estimate, we need to account for the fact that velocity gets integrated into
				// the change of position. we essentially want to remove the area underneath the delta_x curve that results in the velocity drift.
				// the area is a triangle with width i_diff and height v_int_drift, plus a rectangle with width acc_stable_cnt and height v_int_drift.
				// since the deltat value is not constant, we want to use the average deltat (which happens to be sampleFreq) to convert the x-axis from
				// sample number to time in seconds
				delta_x_imu.x -= (v_int_drift.x * ((i_diff / 2) + (float)acc_stable_cnt) / sampleFreq);
				delta_x_imu.y -= (v_int_drift.y * ((i_diff / 2) + (float)acc_stable_cnt) / sampleFreq);
				delta_x_imu.z -= (v_int_drift.z * ((i_diff / 2) + (float)acc_stable_cnt) / sampleFreq);
				v_corrected.x -= v_int_drift.x;
				v_corrected.y -= v_int_drift.y;
				v_corrected.z -= v_int_drift.z;
				acc_stable_flag = 0;  // until the system is unstable again, don't reaccount for velocity drift
			}

			// if the IMU is stable and has already accounted for velocity drift, then save the current iteration as the last stable iteration
			// note: v_corrected_last_stable is always 0 based on how this algorithm is set up (assumes stationary means zero velocity), however
			// it is left in just in case future versions of the code want to assume a "stable IMU" means zero velocity
			else if (!acc_stable_flag){
				last_stable_i = imu_i;
				v_corrected_last_stable.x = v_corrected.x;
				v_corrected_last_stable.y = v_corrected.y;
				v_corrected_last_stable.z = v_corrected.z;
			}
		}

		// integrate the corrected velocity into a change in position
		delta_x_imu.x += v_corrected.x * deltat/1000;
		delta_x_imu.y += v_corrected.y * deltat/1000;
		delta_x_imu.z += v_corrected.z * deltat/1000;
	}

	// prevent the imu counter from going to infinity
	if (imu_i < 9999) {
		imu_i++;
	}
	else{
 		imu_i = 0;
	}

	// if the filter has converged, set the initial quaternion
  if (imu_i == -1){
  	initQuat.w = q0;
  	initQuat.x = -q1;
  	initQuat.y = -q2;
  	initQuat.z = -q3;
  	init_yaw = extract_yaw(initQuat);
  	imu_init_set = 1;
  	imu_i = 0;
  }
}

/**********************************************************************************
 * QUATERNION AND HELPER USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Converts quaternion representation to Euler angle (3-2-1) representation
 * @param q Quaternion representation
 * @return angles Euler angle representation (3-2-1)
 *
 * Source:
 * @see https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
 */
EulerAngles QuaternionToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    float32_t sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float32_t cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // pitch (y-axis rotation)
    float32_t sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1)
        angles.pitch = copysignf(90.0f, sinp) * 180.0f / M_PI; // use 90 degrees if out of range
    else
        angles.pitch = asinf(sinp) * 180.0f / M_PI;

    // yaw (z-axis rotation)
    float32_t siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float32_t cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

    return angles;
}

/**
 * @brief Rotates a 3D point by a quaternion
 * @param point 3D point to be rotated
 * @param q Quaternion to rotate point by
 * @return rotated Rotated 3D point
 *
 * Source:
 * @see https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
 */
Point3D rotatePoint(Point3D point, Quaternion q) {
  Point3D rotated;
  float x = point.x, y = point.y, z = point.z;

	// First multiplication: q * v
	float tw = -q.x*x - q.y*y - q.z*z;
	float tx =  q.w*x + q.y*z - q.z*y;
	float ty =  q.w*y - q.x*z + q.z*x;
	float tz =  q.w*z + q.x*y - q.y*x;

	// Second multiplication: (q * v) * q^-1
	// Note: for a unit quaternion, q^-1 = conjugate of q = (q.w, -q.x, -q.y, -q.z)
	rotated.x = tw*(-q.x) + tx*q.w + ty*(-q.z) - tz*(-q.y);
	rotated.y = tw*(-q.y) - tx*(-q.z) + ty*q.w + tz*(-q.x);
	rotated.z = tw*(-q.z) + tx*(-q.y) - ty*(-q.x) + tz*q.w;

	return rotated;
}

/**
 * @brief Converts Euler angles to their quaternion representation
 * @param angles Euler angle representation
 * @return q Quaternion representation
 *
 * Source:
 * @see https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
 */
Quaternion EulerAnglesToQuaternion(EulerAngles angles) {
    Quaternion q;
    float cy = cosf(angles.yaw * 0.5f * M_PI / 180.0f);
    float sy = sinf(angles.yaw * 0.5f * M_PI / 180.0f);
    float cp = cosf(angles.pitch * 0.5f * M_PI / 180.0f);
    float sp = sinf(angles.pitch * 0.5f * M_PI / 180.0f);
    float cr = cosf(angles.roll * 0.5f * M_PI / 180.0f);
    float sr = sinf(angles.roll * 0.5f * M_PI / 180.0f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

/**
 * @brief Normalizes a quaternion by its magnitude
 * @param q Pointer to quaternion struct
 */
void normalize_quaternion(Quaternion* q) {
    float magnitude = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w /= magnitude;
    q->x /= magnitude;
    q->y /= magnitude;
    q->z /= magnitude;
}

/**
 * @brief Extracts the yaw from a quaternion
 * @param q Quaternion to extract yaw from
 * @return yaw Angle in radians
 *
 * Source:
 * @see https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
 */
float extract_yaw(Quaternion q) {
    return atan2(2.0f * (q.w * q.z + q.x * q.y),
                 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

/**
 * @brief Creates a quaternion from a yaw rotation
 * @param yaw Angle in radians
 * @return q Quaternion with yaw-only rotation
 */
Quaternion create_yaw_quaternion(float yaw) {
    Quaternion q;
    q.w = cos(yaw / 2.0f);
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = sin(yaw / 2.0f);
    return q;
}

/**
 * @brief Multiplies two quaternions together
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return result q1*q2
 *
 * Source:
 * @see https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
 */
Quaternion multiply_quaternions(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

/**
 * @brief Fast inverse square root
 * @param x Input value
 * @return Inverse square root of input value
 *
 * Credit goes to Seb Madgwick
 */
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


/**
 * @brief Generates a random value from a normal distribution
 * @param mean Mean of normal distribution
 * @param stddev Standard deviation of normal distribution
 * @return rand Random value
 *
 * This function implements a linear congruential generator (LCG) and a Box-Mulle transform (BMT) to generate psuedorandom
 * values from a normal distribution. The LCG generates two random values between -1 and 1, and the BMT converts this into
 * random number following a normal distribution.
 *
 * @see https://www.columbia.edu/~ks20/4404-16-Fall/Simulation-LCG.pdf
 * @see https://medium.com/mti-technology/how-to-generate-gaussian-samples-3951f2203ab0
 */
float randn(float mean, float stddev) {
  static uint32_t seed = 1;
  static int have_spare = 0;
  static float spare;
  float u, v, s, mul;

  if (have_spare) {
    have_spare = 0;
    return mean + stddev * spare;
  }

  do {
    // Generate two uniform random numbers between -1 and 1
    seed = 1664525 * seed + 1013904223; // Linear congruential generator
    u = ((float)seed / UINT32_MAX) * 2 - 1;
    seed = 1664525 * seed + 1013904223;
    v = ((float)seed / UINT32_MAX) * 2 - 1;
    s = u * u + v * v;
  } while (s >= 1 || s == 0);

  mul = sqrtf(-2.0f * logf(s) / s);
  spare = v * mul;
  have_spare = 1;

  return mean + stddev * u * mul;
}

/**********************************************************************************
 * KALMAN FILTER USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Creates a Kalman filter struct with given state and measurement dimensions
 * @param state_dim Dimension of state vector
 * @param meas_dim Dimension of measurement vector
 * @return kf Pointer to Kalman filter struct
 *
 * This function
 */
KalmanFilter* create_kalman_filter(size_t state_dim, size_t meas_dim) {
  KalmanFilter* kf = (KalmanFilter*)malloc(sizeof(KalmanFilter));
  if (kf == NULL) return NULL;

  kf->state_dim = state_dim;
  kf->meas_dim = meas_dim;

  kf->x_data = (float32_t*)malloc(state_dim * sizeof(float32_t));
  kf->P_data = (float32_t*)malloc(state_dim * state_dim * sizeof(float32_t));
  kf->F_data = (float32_t*)malloc(state_dim * state_dim * sizeof(float32_t));
  kf->H_data = (float32_t*)malloc(meas_dim * state_dim * sizeof(float32_t));
  kf->Q_data = (float32_t*)malloc(state_dim * state_dim * sizeof(float32_t));
  kf->R_data = (float32_t*)malloc(meas_dim * meas_dim * sizeof(float32_t));
  kf->K_data = (float32_t*)malloc(state_dim * meas_dim * sizeof(float32_t));

  // Initialize matrices
  arm_mat_init_f32(&kf->x, state_dim, 1, kf->x_data);
  arm_mat_init_f32(&kf->P, state_dim, state_dim, kf->P_data);
  arm_mat_init_f32(&kf->F, state_dim, state_dim, kf->F_data);
  arm_mat_init_f32(&kf->H, meas_dim, state_dim, kf->H_data);
  arm_mat_init_f32(&kf->Q, state_dim, state_dim, kf->Q_data);
  arm_mat_init_f32(&kf->R, meas_dim, meas_dim, kf->R_data);
  arm_mat_init_f32(&kf->K, state_dim, meas_dim, kf->K_data);

  return kf;
}

/**
 * @brief Initializes a Kalman filter with given matrices
 * @param kf Pointer to Kalman filter struct
 * @param x_values Pointer to initial x values list
 * @param P_values Pointer to initial P values list
 * @param F_values Pointer to initial F values list
 * @param H_values Pointer to initial H values list
 * @param Q_values Pointer to initial Q values list
 * @param R_values Pointer to initial R values list
 */
void kalman_filter_init(KalmanFilter* kf, float32_t* x_values, float32_t* P_values,
                        float32_t* F_values, float32_t* H_values,
                        float32_t* Q_values, float32_t* R_values) {
  memcpy(kf->x_data, x_values, kf->state_dim * sizeof(float32_t));
  memcpy(kf->P_data, P_values, kf->state_dim * kf->state_dim * sizeof(float32_t));
  memcpy(kf->F_data, F_values, kf->state_dim * kf->state_dim * sizeof(float32_t));
  memcpy(kf->H_data, H_values, kf->meas_dim * kf->state_dim * sizeof(float32_t));
  memcpy(kf->Q_data, Q_values, kf->state_dim * kf->state_dim * sizeof(float32_t));
  memcpy(kf->R_data, R_values, kf->meas_dim * kf->meas_dim * sizeof(float32_t));
}

/**
 * @brief Runs the predict step for a Kalman filter
 * @param kf Pointer to Kalman filter struct
 *
 * This function runs the prediction step for a Kalman filter.
 *
 * For Kalman filter theory, read my thesis text or the source below.
 *
 * @see https://thekalmanfilter.com/kalman-filter-explained-simply/
 */
void kalman_filter_predict(KalmanFilter* kf) {
  // Temporary matrices
  float32_t* temp_state = (float32_t*)malloc(kf->state_dim * sizeof(float32_t));
  float32_t* temp_cov = (float32_t*)malloc(kf->state_dim * kf->state_dim * sizeof(float32_t));
  float32_t* temp_F_transpose = (float32_t*)malloc(kf->state_dim * kf->state_dim * sizeof(float32_t));
  arm_matrix_instance_f32 temp_state_mat, temp_cov_mat, F_transpose_mat;
  arm_mat_init_f32(&temp_state_mat, kf->state_dim, 1, temp_state);
  arm_mat_init_f32(&temp_cov_mat, kf->state_dim, kf->state_dim, temp_cov);
  arm_mat_init_f32(&F_transpose_mat, kf->state_dim, kf->state_dim, temp_F_transpose);

  // x = F * x
  arm_mat_mult_f32(&kf->F, &kf->x, &temp_state_mat);
  memcpy(kf->x_data, temp_state, kf->state_dim * sizeof(float32_t));

  // P = F * P * F^T + Q
  arm_mat_mult_f32(&kf->F, &kf->P, &temp_cov_mat);
  arm_mat_trans_f32(&kf->F, &F_transpose_mat);
  arm_mat_mult_f32(&temp_cov_mat, &F_transpose_mat, &kf->P);
  arm_mat_add_f32(&kf->P, &kf->Q, &kf->P);

  free(temp_state);
  free(temp_cov);
  free(temp_F_transpose);
}

/**
 * @brief Runs the update step for a Kalman filter
 * @param kf Pointer to Kalman filter struct
 * @param measurement Pointer to vector with measurement values
 *
 * This function runs the update step for a Kalman filter.
 *
 * For Kalman filter theory, read my thesis text or the source below.
 *
 * @see https://thekalmanfilter.com/kalman-filter-explained-simply/
 */
void kalman_filter_update(KalmanFilter* kf, float32_t* measurement) {
  // Temporary matrices
  float32_t* temp_state = (float32_t*)malloc(kf->state_dim * sizeof(float32_t));
  float32_t* temp_measure = (float32_t*)malloc(kf->meas_dim * sizeof(float32_t));
  float32_t* temp_H_P = (float32_t*)malloc(kf->meas_dim * kf->state_dim * sizeof(float32_t));
  float32_t* temp_H_transpose = (float32_t*)malloc(kf->state_dim * kf->meas_dim * sizeof(float32_t));
  float32_t* temp_P_H_transpose = (float32_t*)malloc(kf->state_dim * kf->meas_dim * sizeof(float32_t));
  float32_t* temp_S = (float32_t*)malloc(kf->meas_dim * kf->meas_dim * sizeof(float32_t));
  float32_t* temp_S_inv = (float32_t*)malloc(kf->meas_dim * kf->meas_dim * sizeof(float32_t));
  float32_t* temp_K_H = (float32_t*)malloc(kf->state_dim * kf->state_dim * sizeof(float32_t));
  float32_t* temp_K_H_P = (float32_t*)malloc(kf->state_dim * kf->state_dim * sizeof(float32_t));

  arm_matrix_instance_f32 temp_state_mat, temp_measure_mat, temp_H_P_mat, H_transpose_mat, P_H_transpose_mat, S_mat, S_inv_mat, K_H_mat, K_H_P_mat;

  arm_mat_init_f32(&temp_state_mat, kf->state_dim, 1, temp_state);
  arm_mat_init_f32(&temp_measure_mat, kf->meas_dim, 1, temp_measure);
  arm_mat_init_f32(&temp_H_P_mat, kf->meas_dim, kf->state_dim, temp_H_P);
  arm_mat_init_f32(&H_transpose_mat, kf->state_dim, kf->meas_dim, temp_H_transpose);
  arm_mat_init_f32(&P_H_transpose_mat, kf->state_dim, kf->meas_dim, temp_P_H_transpose);
  arm_mat_init_f32(&S_mat, kf->meas_dim, kf->meas_dim, temp_S);
  arm_mat_init_f32(&S_inv_mat, kf->meas_dim, kf->meas_dim, temp_S_inv);
  arm_mat_init_f32(&K_H_mat, kf->state_dim, kf->state_dim, temp_K_H);
  arm_mat_init_f32(&K_H_P_mat, kf->state_dim, kf->state_dim, temp_K_H_P);

  // y = z - H * x
  arm_mat_mult_f32(&kf->H, &kf->x, &temp_measure_mat);
  arm_sub_f32(measurement, temp_measure, temp_measure, kf->meas_dim);

  // S = H * P * H^T + R
  arm_mat_mult_f32(&kf->H, &kf->P, &temp_H_P_mat);
  arm_mat_trans_f32(&kf->H, &H_transpose_mat);
  arm_mat_mult_f32(&temp_H_P_mat, &H_transpose_mat, &S_mat);
  arm_mat_add_f32(&S_mat, &kf->R, &S_mat);

  // K = P * H^T * S^-1
  arm_mat_inverse_f32(&S_mat, &S_inv_mat);
  arm_mat_mult_f32(&kf->P, &H_transpose_mat, &P_H_transpose_mat);
  arm_mat_mult_f32(&P_H_transpose_mat, &S_inv_mat, &kf->K);

  // x = x + K * y
  arm_mat_mult_f32(&kf->K, &temp_measure_mat, &temp_state_mat);
  arm_add_f32(kf->x_data, temp_state, kf->x_data, kf->state_dim);

  // P = (I - K * H) * P
  arm_mat_mult_f32(&kf->K, &kf->H, &K_H_mat);
  arm_mat_mult_f32(&K_H_mat, &kf->P, &K_H_P_mat);
  arm_mat_sub_f32(&kf->P, &K_H_P_mat, &kf->P);

  free(temp_state);
  free(temp_measure);
  free(temp_H_P);
  free(temp_H_transpose);
  free(temp_P_H_transpose);
  free(temp_S);
  free(temp_S_inv);
  free(temp_K_H);
  free(temp_K_H_P);
}

/**
 * @brief Initializes the Kalman filter for the combined system (iSBL + dead reckoning)
 * @return kf Pointer to combined Kalman filter struct
 *
 * This function houses the initial matrices for the combined KF.  It creates a Kalman filter struct with these matrices.
 */
KalmanFilter* init_kalman_filter_combined(void) {
  float32_t x_values[6] = {0, 0, 0, 0, 0, 0};

  float32_t P_values[6 * 6] = {
      10, 0, 0, 0, 0, 0,
      0, 10, 0, 0, 0, 0,
      0, 0, 10, 0, 0, 0,
      0, 0, 0, 10, 0, 0,
      0, 0, 0, 0, 10, 0,
      0, 0, 0, 0, 0, 10
  };

  float32_t F_values[6 * 6] = {
      1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
  };

  float32_t H_values[6 * 6] = {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
  };

  float32_t Q_values[6 * 6] = {
      .1, 0, 0, 0, 0, 0,
      0, .1, 0, 0, 0, 0,
      0, 0, .1, 0, 0, 0,
      0, 0, 0, 10, 0, 0,
      0, 0, 0, 0, 10, 0,
      0, 0, 0, 0, 0, 10
  };

  float32_t R_values[6 * 6] = {
      1000, 0, 0, 0, 0, 0,
      0, 1000, 0, 0, 0, 0,
      0, 0, 1000, 0, 0, 0,
      0, 0, 0, 10000, 0, 0,
      0, 0, 0, 0, 10000, 0,
      0, 0, 0, 0, 0, 10000
  };

  KalmanFilter* kf = create_kalman_filter(6, 6);
  kalman_filter_init(kf, x_values, P_values, F_values, H_values, Q_values, R_values);
  return kf;
}

/**
 * @brief Initializes the Kalman filter for the iSBL-only system
 * @return kf Pointer to combined Kalman filter struct
 *
 * This function houses the initial matrices for the acoustic positioning system-only KF.
 * It creates a Kalman filter struct with these matrices.
 */
KalmanFilter* init_kalman_filter_isbl() {
  float32_t x_values[6] = {0, 0, 0, 0, 0, 0};

  float32_t P_values[6 * 6] = {
      10, 0, 0, 0, 0, 0,
      0, 10, 0, 0, 0, 0,
      0, 0, 10, 0, 0, 0,
      0, 0, 0, 10, 0, 0,
      0, 0, 0, 0, 10, 0,
      0, 0, 0, 0, 0, 10
  };

  float32_t F_values[6 * 6] = {
      1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
  };

  float32_t H_values[3 * 6] = {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0
  };

  float32_t Q_values[6 * 6] = {
      .1, 0, 0, 0, 0, 0,
      0, .1, 0, 0, 0, 0,
      0, 0, .1, 0, 0, 0,
      0, 0, 0, 10, 0, 0,
      0, 0, 0, 0, 10, 0,
      0, 0, 0, 0, 0, 10
  };

  float32_t R_values[3 * 3] = {
      1000, 0, 0,
      0, 1000, 0,
      0, 0, 1000
  };

  KalmanFilter* kf = create_kalman_filter(6, 3);
  kalman_filter_init(kf, x_values, P_values, F_values, H_values, Q_values, R_values);
  return kf;
}

/**
 * @brief Updates the Kalman filter structs with a new delta_time measurement
 * @param kf_combined Pointer to the combined KF struct
 * @param kf_isbl Pointer to the iSBL-only KF struct
 * @param dt Change in time since last acoustic measurement, in seconds
 *
 * This function updates the matrices for both Kalman filter structs to incorporate a non-constant delta_time.
 * The derivations for the Kalman filter matrices can be found in my thesis, but essentially some matrices have
 * state transition matrices and state-to-measurement matrices with a dt term. This allows for updating that term
 * (which is relatively constant, within 3-5 seconds usually).
 */
void update_kalman_matrices(KalmanFilter* kf_combined, KalmanFilter* kf_isbl, float32_t dt) {
  // Update F matrix for all KalmanFilters
  // Assuming F is a 6x6 matrix for all filters
  kf_combined->F_data[3] = dt;
  kf_combined->F_data[10] = dt;
  kf_combined->F_data[17] = dt;

  kf_isbl->F_data[3] = dt;
  kf_isbl->F_data[10] = dt;
  kf_isbl->F_data[17] = dt;

  // Update H matrix for combined KF (6x6)
  kf_combined->H_data[21] = dt;
  kf_combined->H_data[28] = dt;
  kf_combined->H_data[35] = dt;

  // No update needed for ISBL KF H matrix
}

/**********************************************************************************
 * SERIAL COMMUNICATION USER FUNCTIONS
 **********************************************************************************/

/**
 * @brief Transmits a line over UART
 * @param huart Pointer to which UART instance to use
 * @param line Pointer to line of characters to send
 *
 * This function allows for robust data transmission from the STM32 to the ESP32.
 */
void transmit_line(UART_HandleTypeDef *huart, const char *line) {
    HAL_UART_Transmit(huart, (uint8_t*)line, strlen(line), HAL_MAX_DELAY);
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY) {
        // Wait for transmission to complete
    }
    HAL_Delay(75); // Small delay between lines
}

/**
 * @brief Receives a float value over UART with checksums and start/end chars
 * @param huart Point to UART instance to use
 * @param result Pointer to float to store transmitted value in
 * @return status Status of reception
 *
 * This function implements a checksum and start/end characters to receive a float value over a poor UART connection.
 * In testing, the received message kept getting corrupted, and it was very important to have a good value sent.
 */
HAL_StatusTypeDef receiveRobustFloat(UART_HandleTypeDef *huart, float *result) {
    char buffer[MAX_MESSAGE_LENGTH] = {0};
    int bufferIndex = 0;
    int timeout = 2000;  // 1 second timeout
    uint32_t startTime = HAL_GetTick();
    HAL_StatusTypeDef status;
    char receivedChar;

    // clear any leftover data in the buffer
    while (HAL_UART_Receive(huart, (uint8_t*)&receivedChar, 1, 1) == HAL_OK) {
        // Do nothing, just empty the buffer
    }

    // wait for start marker
    do {
        status = HAL_UART_Receive(huart, (uint8_t*)&receivedChar, 1, timeout);
        if (HAL_GetTick() - startTime > timeout) {
            return HAL_TIMEOUT;
        }
    } while (status == HAL_OK && receivedChar != START_MARKER);

    // read until end marker
    do {
        status = HAL_UART_Receive(huart, (uint8_t*)&receivedChar, 1, timeout);
        if (status == HAL_OK && receivedChar != END_MARKER) {
            if (bufferIndex < MAX_MESSAGE_LENGTH - 1) {
                buffer[bufferIndex++] = receivedChar;
            }
        }
        if (HAL_GetTick() - startTime > timeout) {
            return HAL_TIMEOUT;
        }
    } while (status == HAL_OK && receivedChar != END_MARKER);

    buffer[bufferIndex] = '\0';  // null-terminate the string

    // parse the message
    char *commaPos = strchr(buffer, ',');
    if (commaPos == NULL) {
        return HAL_ERROR;  // no comma found, invalid format
    }

    *commaPos = '\0';  // split the string
    char *valueStr = buffer;
    char *checksumStr = commaPos + 1;

    // calculate checksum
    int calculatedChecksum = 0;
    for (int i = 0; valueStr[i] != '\0'; i++) {
        calculatedChecksum += valueStr[i];
    }
    calculatedChecksum %= CHECKSUM_MODULUS;

    // compare calculated and received checksum
    int receivedChecksum = atoi(checksumStr);
    if (calculatedChecksum != receivedChecksum) {
        return HAL_ERROR;  // checksum mismatch
    }

    // parse the float value
    *result = strtof(valueStr, NULL);

    return HAL_OK;
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (imu_rdy){
		if (GPIO_Pin == GPIO_PIN_8)
		{
			lis3mdl_read_data_drdy_handler();
			lsm6dsox_read_data_drdy_handler();
			data_rdy = 1;
			if (imu_calibrated && !recording){
				update_filter();
			}
		}
	}
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

#ifdef  USE_FULL_ASSERT
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
