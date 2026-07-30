/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // 用于 sprintf 格式化输出
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t sbus_rx_buffer[25]; // 存放 SBUS 原始 25 字节
uint16_t channels[16];      // 存放解析后的 16 个通道原始值 (范围 172-1811)
uint16_t pwm_channels[16];  // 存放映射后的 PWM 脉宽值 (范围 1000-2000)
char debug_msg[128];        // 加大缓冲区，彻底防止 sprintf 溢出
uint8_t is_failsafe = 0;
static uint16_t sync_cycle_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 1. SBUS 数值映射函数 (将 172~1811 转换为 1000~2000)
uint16_t map_sbus_to_pwm(uint16_t sbus_val) {
    long pwm = ((long)(sbus_val - 172) * 1000 / 1639) + 1000;
    // 限制死区，防止电调/舵机越界
    if (pwm < 800)  return 800;
    if (pwm > 2200) return 2200;
    return (uint16_t)pwm;
}
// CRC-8, 多项式 0x31, init 0x00 (与 PX4 / mocap 一致)
static uint8_t telem_crc8(const uint8_t *ptr, uint16_t len)
{
    uint8_t crc = 0x00u;
    while (len--) {
        crc = (uint8_t)(crc ^ *ptr++);
        for (uint8_t i = 0; i < 8u; i++) {
            if (crc & 0x80u) {
                crc = (uint8_t)(((uint8_t)(crc << 1)) ^ 0x31u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t Get_CRC8(uint8_t *ptr, uint16_t len)
{
    return telem_crc8(ptr, len);
}

// 2. 升级版 SBUS 解析函数：带自动对齐 + 映射 + 状态指示
uint8_t Decode_SBUS(uint8_t *buf) {
    // 遍历整个 25 字节缓冲区，寻找帧头 0x0F 和 帧尾 0x00
    for (int i = 0; i < 25; i++) {
        // 使用取模运算 %25 保证跨越缓冲区边界也能正确读取
        if (buf[i] == 0x0F && buf[(i + 24) % 25] == 0x00) {

            // 找到有效帧头，开始按偏移量提取通道数据
            channels[0]  = ((buf[(i+1)%25]    | buf[(i+2)%25]<<8)                       & 0x07FF);
            channels[1]  = ((buf[(i+2)%25]>>3 | buf[(i+3)%25]<<5)                       & 0x07FF);
            channels[2]  = ((buf[(i+3)%25]>>6 | buf[(i+4)%25]<<2 | buf[(i+5)%25]<<10)   & 0x07FF);
            channels[3]  = ((buf[(i+5)%25]>>1 | buf[(i+6)%25]<<7)                       & 0x07FF);
            channels[4]  = ((buf[(i+6)%25]>>4 | buf[(i+7)%25]<<4)                       & 0x07FF);
            channels[5]  = ((buf[(i+7)%25]>>7 | buf[(i+8)%25]<<1 | buf[(i+9)%25]<<9)    & 0x07FF);
            channels[6]  = ((buf[(i+9)%25]>>2 | buf[(i+10)%25]<<6)                      & 0x07FF);
            channels[7]  = ((buf[(i+10)%25]>>5| buf[(i+11)%25]<<3)                      & 0x07FF);
            channels[8]  = ((buf[(i+12)%25]   | buf[(i+13)%25]<<8)                      & 0x07FF);
            channels[9]  = ((buf[(i+13)%25]>>3| buf[(i+14)%25]<<5)                      & 0x07FF);
            channels[10] = ((buf[(i+14)%25]>>6| buf[(i+15)%25]<<2| buf[(i+16)%25]<<10)  & 0x07FF);
            channels[11] = ((buf[(i+16)%25]>>1| buf[(i+17)%25]<<7)                      & 0x07FF);
            channels[12] = ((buf[(i+17)%25]>>4| buf[(i+18)%25]<<4)                      & 0x07FF);
            channels[13] = ((buf[(i+18)%25]>>7| buf[(i+19)%25]<<1| buf[(i+20)%25]<<9)   & 0x07FF);
            channels[14] = ((buf[(i+20)%25]>>2| buf[(i+21)%25]<<6)                      & 0x07FF);
            channels[15] = ((buf[(i+21)%25]>>5| buf[(i+22)%25]<<3)                      & 0x07FF);

            // 【新增逻辑】将提取出的原始数值映射为 1000-2000 的 PWM 脉宽
            for(int j = 0; j < 16; j++){
                pwm_channels[j] = map_sbus_to_pwm(channels[j]);
            }

            // 【新增】解析第 24 字节（标志位）
            uint8_t flags = buf[(i + 23) % 25];

            // 提取失控保护位 (0x08)
            if ((flags & 0x08) != 0) {
                is_failsafe = 1; // 遥控器已关机
            } else {
                is_failsafe = 0; // 遥控器连接正常
            }

            // 【修改 LED 逻辑】
            if (is_failsafe) {
                // 如果失控，让灯常灭（或常亮，取决于你的硬件），代表报警！
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            } else {
                // 如果正常连接，灯才继续闪烁
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // 【关键修复 1：阅后即焚】
            // 破坏掉刚解析完的帧头，防止 DMA 停止后死循环读取这一帧旧数据！
            buf[i] = 0x00;
            return 1; // 成功解析一帧，返回 1
        }
    }
    return 0; // 找不到有效帧，返回 0
}

uint8_t px4_tx_buffer[24];

// Data[12] 线上加扰：与 BDDB 交替 XOR，避免开关通道为 0 时出现长串 00
static const uint8_t TELEM_WHITEN[12] = {
    0xBD, 0xDB, 0xBD, 0xDB, 0xBD, 0xDB,
    0xBD, 0xDB, 0xBD, 0xDB, 0xBD, 0xDB
};

/**
 * 96bit 压缩 RC @ 15Hz / 9600
 * 帧: 00 | AD×5 | FE | 10 | CRC | Data'[12] | BD DB BD
 * Data' = Data XOR BDDB…；CRC-8(0x31) 仅覆盖 Data'
 */
uint8_t Compress_And_Send_PX4_Protocol(void) {
    static uint32_t last_tx_tick = 0;
    if (HAL_GetTick() - last_tx_tick < 66) { // 15 Hz
        return 0;
    }
    last_tx_tick = HAL_GetTick();

    uint16_t vals_11b[4];
    uint8_t  vals_6b[6];

    for (int i = 0; i < 4; i++) {
        long norm = ((long)(channels[i] - 172) * 2047) / (1811 - 172);
        if (norm < 0) norm = 0;
        if (norm > 2047) norm = 2047;
        vals_11b[i] = (uint16_t)norm;
    }

    for (int i = 4; i < 10; i++) {
        uint16_t val = channels[i];
        uint8_t res_val = 0;
        if (i == 9) {
            uint16_t pwm_val = pwm_channels[9];
            if (pwm_val < 1600) {
                res_val = 0;
            } else {
                res_val = 63;
            }
        } else if (i == 5 || i == 6) {
            if (val < 600) res_val = 0;
            else if (val > 1400) res_val = 63;
            else res_val = 31;
        } else {
            if (val < 992) res_val = 0;
            else res_val = 63;
        }
        vals_6b[i - 4] = res_val;
    }

    sync_cycle_counter = (sync_cycle_counter + 1) & 0x07FF;

    uint8_t action_flag = 0;
    if (channels[4] > 1400) {
        action_flag = 1;
    }

    uint64_t p_low = 0;
    uint32_t p_high = 0;

    p_low |= (uint64_t)vals_11b[0];
    p_low |= ((uint64_t)vals_11b[1] << 11);
    p_low |= ((uint64_t)vals_11b[2] << 22);
    p_low |= ((uint64_t)vals_11b[3] << 33);
    p_low |= ((uint64_t)vals_6b[0]  << 44);
    p_low |= ((uint64_t)vals_6b[1]  << 50);
    p_low |= ((uint64_t)vals_6b[2]  << 56);
    p_low |= ((uint64_t)(vals_6b[3] & 0x03) << 62);

    p_high |= ((uint32_t)(vals_6b[3] >> 2));
    p_high |= ((uint32_t)vals_6b[4] << 4);
    p_high |= ((uint32_t)vals_6b[5] << 10);
    p_high |= ((uint32_t)sync_cycle_counter << 16);
    p_high |= ((uint32_t)action_flag << 27);

    uint8_t data_bytes[12];
    for (int i = 0; i < 8; i++) {
        data_bytes[i] = (uint8_t)((p_low >> (8 * i)) & 0xFFu);
    }
    for (int i = 0; i < 4; i++) {
        data_bytes[8 + i] = (uint8_t)((p_high >> (8 * i)) & 0xFFu);
    }

    // 线上加扰
    for (int i = 0; i < 12; i++) {
        data_bytes[i] ^= TELEM_WHITEN[i];
    }

    const uint8_t header[8] = {0x00, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xFE, 0x10};
    for (int i = 0; i < 8; i++) {
        px4_tx_buffer[i] = header[i];
    }
    for (int i = 0; i < 12; i++) {
        px4_tx_buffer[9 + i] = data_bytes[i];
    }
    px4_tx_buffer[8] = telem_crc8(&px4_tx_buffer[9], 12);

    // 帧尾 pad: BD DB BD
    px4_tx_buffer[21] = 0xBD;
    px4_tx_buffer[22] = 0xDB;
    px4_tx_buffer[23] = 0xBD;

    if (HAL_UART_Transmit(&huart2, px4_tx_buffer, 24, 100) == HAL_OK) {
        return 1;
    }
    return 0;
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  // 初始化 PC13 为推挽输出（用于板载 LED）
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END Init */

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init(); // SBUS 接收串口 (100000 波特率)
  MX_USART2_UART_Init(); // PX4 转发串口 (9600 波特率)

  /* USER CODE BEGIN 2 */

  // 启动串口 1 (SBUS) 的 DMA 循环接收模式
  HAL_UART_Receive_DMA(&huart1, sbus_rx_buffer, 25);

  uint32_t last_sbus_time = HAL_GetTick(); // 记录上一次成功收到有效 SBUS 的时间

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 解析到有效 SBUS 帧
    if (Decode_SBUS(sbus_rx_buffer) == 1) {
        last_sbus_time = HAL_GetTick();

        // 2. 未失控则按 15Hz 压缩转发
        if (is_failsafe == 0) {
            if (Compress_And_Send_PX4_Protocol() == 1) {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }
    }

    // 3. 超时/失控：灯停闪
    if ((HAL_GetTick() - last_sbus_time > 500) || (is_failsafe == 1)) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }

    HAL_Delay(1);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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

  // 使用内部 HSI (8MHz)，避免外部晶振频率/焊接异常导致 UART 波特率整体偏移
  // HSI/2 = 4MHz, PLL×12 = 48MHz (与原先 HSE 8MHz×6 主频一致，USART 分频不变)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
