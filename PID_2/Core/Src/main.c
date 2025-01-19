/* USER CODE BEGIN Header */

// The MPU6050_lib.h library provides a set of functions for easy communication with the MPU6050 sensor.
// By using this library, complex operations such as reading raw data and converting it to scaled values
// are abstracted into straightforward function calls, reducing the need for low-level code and simplifying
// sensor interaction.
#include "MPU6050_lib.h"  	// Include the MPU6050 library for simplified communication
#define Calibrate 1		  	// Calibrate the brushless motor

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include <stdio.h>			// Standard I(O for debugging
#include <math.h>			// Standard library for mathematical functions
/* USER CODE END Includes */

// No typedefs, private defines, or macros used in this project

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//IMU Variables
RawData_Def Accel_Raw, Gyro_Raw;			// Raw data structures to store Accel, Gyro data
ScaledData_Def Accel_Scaled, Gyro_Scaled;	// Scaled data structure to store processed Accel, Gyro data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

// No private function prototypes implemented for this project

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU_Configuration(void){
	  MPU_ConfigTypeDef MPU_Config;	//IMU Configuration

	  //MPU6050 CONFIGURATION
	  MPU_Config.Clock_Source = INTERNAL_8MHz;
	  MPU_Config.Config_Dlpf = DLPF_5_Hz;
	  MPU_Config.Gyro_Full_Scale = FS_SEL_250;
	  MPU_Config.Accel_Full_Scale = AFS_SEL_2g;
	  MPU_Config.Sleep_Mode_Bit = 0;  				// 1: sleep mode, 0: normal mode
	  MPU6050_Init(&hi2c1, &MPU_Config); 			// Initialization - Configuration
}

#define PWR_MGMT_1 0x6B  				// MPU6050 power management register address
#define MPU_ADDR 0x68    				// MPU6050 I2C address

void Reset_MPU6050() {
    I2C_Write8(PWR_MGMT_1, 0x80);	    // Write 0x80 to the PWR_MGMT_1 register to reset the sensor
    HAL_Delay(100);  					// Delay for 100ms to ensure the sensor resets
    I2C_Write8(PWR_MGMT_1, 0x00);		// After the reset, clear the sleep mode by writing 0x00 to PWR_MGMT_1
}

// TIMER CONFIGURATION FOR PWM
void TIM1_PWM_setup() {

    /* TIM1 SetUp */
    RCC->APB2ENR |= (1 << RCC_APB2ENR_TIM1EN_Pos);     // Enable clock to TIM1

    /* GPIO SetUp (PA8 -> TIM1_CH1) */
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);    // Enable GPIOA clock
    GPIOA->MODER |= (2 << 16);                         // Set PA8 to Alternate function mode
    GPIOA->AFR[1] |= (1 << 0);                         // Set AF1 (TIM1_CH1) for PA8
    GPIOA->OSPEEDR |= (3 << 16);                       // High Speed for PA8

    /* CR1 Configuration */
    TIM1->CR1 &= ~(1 << TIM_CR1_UDIS_Pos);             // Update event enabled
    TIM1->CR1 &= ~(1 << TIM_CR1_URS_Pos);              // Generation of an update interrupt
    TIM1->CR1 &= ~(1 << TIM_CR1_DIR_Pos);              // Upcounting direction
    TIM1->CR1 &= ~(0x03 << TIM_CR1_CMS_Pos);           // Edge-aligned mode
    TIM1->CR1 |= (1 << TIM_CR1_ARPE_Pos);              // Auto-reload preload enabled

    /* Channel 1 Configuration */
    TIM1->CCMR1 &= ~(0x03 << TIM_CCMR1_CC1S_Pos);      // TIM1_CH1 configured as output
    TIM1->CCMR1 |= (1 << TIM_CCMR1_OC1PE_Pos);         // Enable preload for CCR1
    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);        // PWM Mode 1 (Active if CNT < CCR1)

    TIM1->CCER |= (1 << TIM_CCER_CC1E_Pos);            // Enable output for channel 1
    TIM1->CCER &= ~(1 << TIM_CCER_CC1P_Pos);           // Active high (default)

    TIM1->PSC = 163 - 1;                               // Set prescaler for 1 kHz timer clock
    TIM1->ARR = 10306 - 1;                             // Set auto-reload for a 50 Hz PWM signal
    TIM1->CCR1 = 0;                                    // 0

    TIM1->EGR |= (1 << TIM_EGR_UG_Pos);                // Generate an update event

    /* Enable the output for the TIM1 channel */
    TIM1->BDTR |= (1 << TIM_BDTR_MOE_Pos);             // Main output enable (necessary for TIM1)
    TIM1->CR1 |= (1 << TIM_CR1_CEN_Pos);               // Enable the counter
}

// Configure TIMER 2 to generate an update event every 0.01 seconds
void TIM2_PID_setup(void) {

    RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM2EN_Pos);	// Enable the clock for TIM2 peripheral
    TIM2->PSC = 8400 - 1;					// Set prescaler to slow the 84 MHz clock down to 10 kHz (84 MHz / 8400)
    TIM2->ARR = 100 - 1;					// Set auto-reload register to 100 for a 0.01-second interval (10 kHz / 100 = 100 Hz or 0.01 s)
    TIM2->DIER |= (1 << TIM_DIER_UIE_Pos);	// Enable update interrupt on timer overflow
    TIM2->CR1 |= (1 << TIM_CR1_CEN_Pos);	// Start the timer by enabling the counter
    NVIC_SetPriority(TIM2_IRQn, 0);			// Set priority level 0 for TIM2 interrupt
    NVIC_EnableIRQ(TIM2_IRQn);				// Enable the TIM2 interrupt in the NVIC to handle the timer events
}

// ISR for Timer 2, triggered every 0.01 seconds based on timer overflow
volatile uint8_t Tc_flag = 0;					//Control Loop Flag

void TIM2_IRQHandler(void) {					// TIM2 Interrupt Handler - Called when a timer overflow occurs for TIM2
    if (TIM2->SR & TIM_SR_UIF) {				// Check if the update interrupt flag (UIF) is set (timer overflowed)
        TIM2->SR &= ~(1 << TIM_SR_UIF_Pos);	    // Clear the update interrupt flag to allow the next interrupt
        Tc_flag = 1;	        				// Set flag to signal the control loop to run
    }
}

// USART2 Configuration
void Manual_USART2_Init(void) {

    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);	// Enable GPIOA Clock

    /* Configure PA2 (TX) */
    GPIOA->MODER |= (0x02 << 4);        // Set PA2 to Alternate Function Mode
    GPIOA->AFR[0] |= (0x07 << 8);       // Set Alternate Function 7 (USART2 TX)
    GPIOA->PUPDR &= ~(0x03 << 4);       // Clear PA2 Pull-Up/Down bits
    GPIOA->PUPDR |= (0x01 << 4);        // Enable Pull-Up for PA2

    /* Configure PA3 (RX) */
    GPIOA->MODER |= (0x02 << 6);        // Set PA3 to Alternate Function Mode
    GPIOA->AFR[0] |= (0x07 << 12);      // Set Alternate Function 7 (USART2 RX)
    GPIOA->PUPDR &= ~(0x03 << 6);       // Clear PA3 Pull-Up/Down bits
    GPIOA->PUPDR |= (0x01 << 6);        // Enable Pull-Up for PA3

    /* Enable USART2 Clock */
    RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);

    /* Configure USART2 Registers */
    USART2->CR1 |= (0x01 << USART_CR1_UE_Pos);		// Enable USART (UE)
    USART2->CR1 |= (0x01 << USART_CR1_M_Pos);		// Define word length (M)
    USART2->CR2 |= (0x02 << USART_CR2_STOP_Pos);	// Define number of stop bits (STOP)
    USART2->CR1 |= (0x1 << USART_CR1_PCE_Pos); 		// Enable parity check (PCE)
    USART2->CR1 &= ~(0x01 << USART_CR1_PS_Pos);		// Even parity (PS)
    USART2->BRR = 0x1117; 							// For 42 MHz clock and 9600 baud rate (Oversampling 16)

    // Enable Transmit and Receive
    USART2->CR1 |= (1 << USART_CR1_TE_Pos); 		// Enable Transmitter
    USART2->CR1 |= (1 << USART_CR1_RE_Pos); 		// Enable Receiver

    /* Enable RXNE Interrupt */
    USART2->CR1 |= (1 << USART_CR1_RXNEIE_Pos);

    /* Enable USART2 in NVIC */
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1); 				// Set second priority
}


// Handling USART2 Interruption for receiving and transmitting data

volatile uint8_t rx_len, tx_len, cmd_received;
uint8_t tx_len_max;

volatile char buff_tx[100];      			// Transmission buffer
volatile char  data_buff[100];    			// Reception buffer
volatile uint8_t tx_len = 0;     			// Counter to keep track of the current byte being transmitted
volatile uint8_t new_data_available = 0; 	// 1 when new data is available, 0 otherwise

void USART2_IRQHandler(void) {
    NVIC_ClearPendingIRQ(USART2_IRQn);	    			// Clear the pending interrupt flag for USART2

    if ((USART2->SR >> USART_SR_RXNE_Pos) & 0x01) {	    // Check if the receive data register is not empty (RXNE)
        	data_buff[0] = (char) USART2->DR;			// Read the received data
        	new_data_available = 1; 					// Set the flag indicating new data is available
    }

    if ((USART2->SR >> USART_SR_TXE_Pos) & 0x01) {		// Check if the transmit data register is empty (TXE)
        if (tx_len < tx_len_max) {						// If there are still bytes to send, load the next byte from buff_tx
            USART2->DR = buff_tx[++tx_len];
        } else {
            // All data has been transmitted, wait for the transmission to complete (TC)
            USART2->CR1 &= ~(0x01 << USART_CR1_TXEIE_Pos); // Disable TXE interrupt
        }
    }

    if ((USART2->SR >> USART_SR_TC_Pos) & 0x01) {			// Check if the transmission is complete (TC)
        tx_len = 0; 										// Reset the transmission length counter
        USART2->CR1 &= ~(0x01 << USART_CR1_TXEIE_Pos); 		// Disable TXE interrupt if it’s still enabled
    }
}

// Function to send a string via USART using interrupts
void send_str_it(volatile char *buff, uint8_t len) {
    tx_len = 0;                						// Reset the transmission counter
    tx_len_max = len - 1;       					// Set the maximum length of data to be sent
    buff_tx[0] = buff[0];       					// Load the first byte into the transmit buffer
    USART2->DR = buff_tx[0];    					// Start transmitting the first byte
    USART2->CR1 |= (0x01 << USART_CR1_TXEIE_Pos); 	// Enable TXE interrupt to send the rest of the data
}


// Function to get the tilt angle from the MPU6050
	/*
		a=tau / (tau + loop time)
		newAngle = angle measured with atan2 using the accelerometer
		newRate = angle measured using the gyro
		looptime = loop time in millis()
	*/
float tau=0.1;		// Time constant for the complementary filter
float a=0.0;		// Filter coefficient
float x_angleC = 0;	// Complementary angle

// Function to calculate the tilt angle using accelerometer and gyroscope data
float Get_Angle_Inclination() {
    // Read raw and scaled data from the MPU6050 sensor
    MPU6050_Read_RawData(&Accel_Raw, &Gyro_Raw);
    MPU6050_Read_ScaledData(&Accel_Scaled, &Gyro_Scaled);

    // Extract accelerometer data
    float accelY = Accel_Scaled.y;
    float accelZ = Accel_Scaled.z;

    // Compute tilt angle from accelerometer data
    float newAngle = (atan2(accelY, accelZ) * (180.0 / M_PI)) + 90.0f;

    // Get gyroscope angular velocity
    float newRate = Gyro_Scaled.x;
    float dtC = 0.01;  // Time interval (seconds)

    // Compute filter coefficient
    a = tau / (tau + dtC);		// 0.9

    // Apply complementary filter to combine accelerometer and gyroscope data
    x_angleC = a * (x_angleC + newRate * dtC) + (1 - a) * newAngle;

    // Clamp the angle to ensure it remains within bounds
    if (x_angleC < 0.0f) x_angleC = 0.0f;
    if (x_angleC > 1.0f) x_angleC = x_angleC;

    return x_angleC;
}

// RELATIONSHIP BRUSHLESS MOTOR A2212 - MAX RPM
#define MAX_RPM 12000  // Maximum RPM at full throttle with 12V supply (1000 KV * 12V)
#define K_t 0.00955    // Torque constant in Nm/A
// PWM LIMITS (1ms to 2ms)
#define MIN_PWM 598    // Minimum PWM value (corresponding to minimum duty cycle - 1ms pulse width)
#define MAX_PWM 646    // Maximum PWM value (corresponding to maximum duty cycle - 2ms pulse width)

// Function to map the control input (voltage) to a PWM signal
float voltage_to_pwm(float voltage){
    if (voltage > MAX_RPM) voltage = MAX_RPM;  // Clamp RPM to max RPM
    if (voltage < 0) voltage = 0;              // Clamp RPM to min RPM
    return (uint32_t)((voltage / MAX_RPM) * (MAX_PWM - MIN_PWM) + MIN_PWM);	// Map the RPM to the PWM range [MIN_PWM, MAX_PWM]
}

/* PID Variables */
volatile float setpoint = 0;               // Target value for the PID controller (angular position)
volatile float process_variable = 0.0;     // Current feedback from the sensor
volatile float control_output = 0.0;       // Output of the PID controller

// PID coefficients (by Ziegler–Nichols method)
float Kp = 78;		// Proportional gain
float Ki = 142;     // Integral gain 142
float Kd = 10;		// Derivative gain

float previous_error = 0.0; // Error value from the previous iteration (for derivative calculation)
float integral = 0.0;       // Accumulated integral of the error (for integral calculation)

/* Function to implement PID control */
float PID_Control(float setpoint, float process_variable) {
    float error = setpoint - process_variable;	    					// Calculate the error between the desired setpoint and the current process variable
    integral += error * 0.01; 											// Integral term accumulates the error over time
    float derivative = (error - previous_error) / 0.01; 				// Derivative term estimates how quickly the error is changing
    control_output = Kp * error + Ki * integral + Kd * derivative;	    // control_output combines proportional, integral, and derivative terms
    uint32_t pwm_value = voltage_to_pwm(control_output);		    	// Compute the PWM value from the voltage required
    previous_error = error;				    							// Store the current error to use as the previous error in the next iteration
    return pwm_value;			    									// Return the PWM value that will be used to control the motor
}

float y, u = 0;							// Input and Output set as Global variables for debugging

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  TIM1_PWM_setup();      		// Set up TIM1 PWM
  TIM2_PID_setup();      		// Set up TIM2 for PID calculation with ISR
  MX_GPIO_Init();				// GIPO
  MX_I2C1_Init();				// I2C
  Manual_USART2_Init();			// USART2 Configuration
  MPU_Configuration();			// Sensor Configuration
  Reset_MPU6050();				// Reset MPU6050 Sensor
  /* USER CODE BEGIN 2 */

// Calibration of the brushless motor
#if Calibrate
  TIM1->CCR1 = 700;  // Set the maximum pulse (2ms)
  HAL_Delay (1000);  // wait for 1 beep
  TIM1->CCR1 = 500;  // Set the minimum Pulse (1ms)
  HAL_Delay (1000);  // wait for 2 beeps
  TIM1->CCR1 = 0;    // reset to 0, so it can be controlled via ADC
#endif

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Main loop to process incoming commands and execute PID control
  while (1) {
      // Check if new data has been received via USART
      if (new_data_available) {
    	  // Execute control logic when new data is available and the timer interrupt has fired (every 0.01 s)
          if (data_buff[0] == '1' && Tc_flag == 1) {

        	  float r = 45;								  // Desired setpoint (target angle)
              y = Get_Angle_Inclination();	              // Measure the current angle
              u = PID_Control(r, y);					  // Compute the control effort using PID
              TIM1->CCR1 = u;							  // Apply the control effort by setting the PWM duty cycle
              sprintf(buff_tx, "%1.3f\r\n", y);			  // Format the measured angle and send it via USART for feedback
              send_str_it(buff_tx, strlen(buff_tx));
              Tc_flag = 0;								  // Reset the control flag for the next cycle
          }
          else if (data_buff[0] == '0' && Tc_flag == 1) { // Command '0': Set motor PWM to a fixed value if Tc_flag is set

              TIM1->CCR1 = 500;							  // Set PWM to a fixed value (Stop the motor)
              Tc_flag = 0;								  // Reset the control flag
          }
      }
  }
    /* USER CODE END WHILE */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */


/*
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
