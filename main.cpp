#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <cstdio>
#include <cstdint>
#include <string.h>
#include <math.h>

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1; // Handle for the HM-10 Bluetooth
ADC_HandleTypeDef hadc1; 

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

volatile float limit1 = 1.0f, limit2 = 1.0f, limit3 = 1.0f;
volatile float Sensor1Norm = 0.0f, Sensor2Norm = 0.0f, Sensor3Norm = 0.0f;
volatile bool paused = true;
volatile int overextensionCount = 0;
uint8_t rxBuffer[1]; // Buffer to hold the incoming character

extern "C" {
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
  }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
      printf("BT RX: %d ('%c')\r\n", rxBuffer[0], rxBuffer[0]);
        if (rxBuffer[0] == 'C' || rxBuffer[0] == 67 || rxBuffer[0] == 'c' || rxBuffer[0] == '99') {
          printf("Calibrating! S1=%.3f S2=%.3f S3=%.3f\r\n", Sensor1Norm, Sensor2Norm, Sensor3Norm);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // LED feedback

            if (Sensor1Norm > 0.1f) limit1 = Sensor1Norm;
            if (Sensor2Norm > 0.1f) limit2 = Sensor2Norm;
            if (Sensor3Norm > 0.1f) limit3 = Sensor3Norm;
        }
        // Re-prime the receive interrupt for the next byte
        else if (rxBuffer[0] == 'P') {  // Pause
          paused = true;
        }
        else if (rxBuffer[0] == 'R') {  // Resume
          paused = false;
        }
        else if (rxBuffer[0] == 'X') {  // Reset
          overextensionCount = 0;
        }
        
        HAL_UART_Receive_IT(&huart1, rxBuffer, 1);
    }
  }
}

class Motor { 
protected: 
    GPIO_TypeDef* Port;
    uint32_t Pin, offTime;
    bool status; 

public: 
    Motor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) : Port(GPIOx), Pin(GPIO_Pin) { off(); } 
    void on(){ 
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET); 
        status = true; 
    } 

    void off(){ 
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET); 
        status = false; 
    } 

    void trigger(uint32_t durationMS) {
        offTime = HAL_GetTick() + durationMS;
        on();
    }

    // This must be called every loop iteration
    void update() {
        if (HAL_GetTick() >= offTime) {
            off();
        } else {
            on();
        }
    }

    void pulse(int currentTime, int period) { 
        if ((currentTime % period) < (period / 2)) on();
        else off();
    } 

    bool getStatus() { return status; } 
}; 

class Sensor { 
private:
    ADC_HandleTypeDef* hadc;
    uint32_t channel;
    float VDD;

    uint16_t readRaw() {
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = channel;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 0;

        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
            uint16_t val = HAL_ADC_GetValue(hadc);
            HAL_ADC_Stop(hadc); 
            return val;
        }
        return 0;
    }
public:
    Sensor(ADC_HandleTypeDef* adcHandle, uint32_t adcChannel, float v) : hadc(adcHandle), channel(adcChannel), VDD(v) {} 

    float amplitudeVolts() { 
      return (static_cast<float>(readRaw()) * VDD / 4095.0f);} 

    uint16_t amplitudeRaw() { 
      return readRaw();}

    float amplitudeNorm() { 
      return static_cast<float>(readRaw()) / 4095.0f;} 
};

void sendBluetoothData(float angle, int overextCount) {
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "%.2f,%d\n", angle, overextCount);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100); 
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR); // Enable error interrupts
    HAL_UART_Receive_IT(&huart1, rxBuffer, 1);   // Re-prime the pump

  Motor Vibration(GPIOB, GPIO_PIN_3);
  Sensor Sensor1(&hadc1, ADC_CHANNEL_0, 3.3f);
  Sensor Sensor2(&hadc1, ADC_CHANNEL_1, 3.3f);
  Sensor Sensor3(&hadc1, ADC_CHANNEL_4, 3.3f);

  const int BUFFER_SIZE = 30;
  float history[BUFFER_SIZE] = {0};
  int index = 0;
  float threshold = 0.2f; // threshold of how much of the range of sensor motion can be changed within 0.03s before motor activated
  int totalMS = 0; // used to track position in intermittent vibration motor cycle
  bool overextended = false;
  int transmitCounter = 0;

  Vibration.off();

  HAL_Delay(500);

 // limit1 = Sensor1.amplitudeNorm();
 // limit2 = Sensor2.amplitudeNorm();
 // limit3 = Sensor3.amplitudeNorm();

  HAL_UART_Receive_IT(&huart1, rxBuffer, 1); // Listen for 1 byte on the Bluetooth

  while (1)
  {
    Sensor1Norm = Sensor1.amplitudeNorm();
    Sensor2Norm = Sensor2.amplitudeNorm();
    Sensor3Norm = Sensor3.amplitudeNorm();

    if (!paused){

    float angle = 90 + (Sensor1Norm * (90/limit1));

     float currentVal = Sensor1.amplitudeNorm();
        float oldestVal = history[index]; 
        
        float delta = fabsf(currentVal - oldestVal); // Calculate the absolute change in sensor value over 0.03s
        
         printf("VDelta: %.3f | S1: %f | S2: %f | S3: %f | L1: %f | L2: %f | L3: %f\r\n", delta, Sensor1Norm, Sensor2Norm, Sensor3Norm, limit1, limit2, limit3); // Print for debugging

        history[index] = currentVal; // replace oldest value in buffer with current value
        index = (index + 1) % BUFFER_SIZE; // move to next oldest value in buffer, loops back to 0 when index hits buffer size
        Vibration.update();

        if (delta > threshold && 0.5 > delta) { 
            Vibration.trigger(1000); // vibration constantly on for rapid movement
        } 

        else if (Sensor1Norm > limit1){
             float ratio1 = (Sensor1Norm - limit1) / (0.07f); // (current - min) / (max - min) outputs between 0 and 1
                if (ratio1 > 1){
                  ratio1 = 1;
                }
              int dynamicPeriod1 = 200 - (int)(ratio1 * 100.0f); // scales and offsets the ratio to make the period be between 100 and 400
              Vibration.pulse(totalMS, dynamicPeriod1); // intermittent buzzing using defined dynamic period

            if (overextended == false){
            overextensionCount++;
            }
            overextended = true;
        } 

        else if (Sensor2Norm > limit2){
              float ratio2 = (Sensor2Norm - limit2) / (0.07f); // (current - min) / (max - min) outputs between 0 and 1
                  if (ratio2 > 1){
                    ratio2 = 1;
                  }
                int dynamicPeriod2 = 200 - (int)(ratio2 * 100.0f); // scales and offsets the ratio to make the period be between 100 and 400
                Vibration.pulse(totalMS, dynamicPeriod2); // intermittent buzzing using defined dynamic period

            if (overextended == false){
            overextensionCount++;
            }
            overextended = true;
        } 

        else if (Sensor3Norm > limit3){
          float ratio3 = (Sensor3Norm - limit3) / (0.07f); // (current - min) / (max - min) outputs between 0 and 1
                if (ratio3 > 1){
                  ratio3 = 1;
                }
              int dynamicPeriod3 = 200 - (int)(ratio3 * 100.0f); // scales and offsets the ratio to make the period be between 100 and 400
              Vibration.pulse(totalMS, dynamicPeriod3); // intermittent buzzing using defined dynamic period
        
            if (overextended == false){
            overextensionCount++;
            }
            overextended = true;
       } 

        else {
            Vibration.off(); 
            overextended = false;
        }
        
        totalMS++; //increase counter for current time in intermittent vibration motor cycle

        if (transmitCounter++ >= 5) { // Send data every 50ms
          sendBluetoothData(angle, overextensionCount);
          transmitCounter = 0;
        }
      }

      else {
        Vibration.off();
      }
    HAL_Delay(10);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600; // HM-10 default
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Clear error flags and restart listening
        HAL_UART_Receive_IT(&huart1, rxBuffer, 1);
    }
}