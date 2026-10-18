#include "stm32f1xx_hal.h"
#include <string.h>

void Error_Handler(void);   /* 错误处理函数的前置声明 */

/*==================== 发送间隔（毫秒）====================
 *   0   = 不加延时（观察丢数据现象）
 *   100 = 加 100ms 延时（数据完整）
 */
#define SEND_DELAY_MS       0

/*---------------- 串口配置参数 ----------------*/
#define UART_BAUDRATE       115200U      /* 波特率 */
#define UART_TIMEOUT        100U         /* 发送超时（毫秒）*/

/*---------------- SysTick 寄存器（延时用，不依赖中断）----------------*/
#define SYSTICK_CTRL    (*(volatile uint32_t *)0xE000E010)
#define SYSTICK_LOAD    (*(volatile uint32_t *)0xE000E014)
#define SYSTICK_VAL     (*(volatile uint32_t *)0xE000E018)

/*---------------- 全局变量 ----------------*/
static UART_HandleTypeDef huart1;             /* USART1 句柄 */
static uint8_t g_SendFlag = 1;                /* 1 = 允许发送，0 = 已暂停 */
static uint8_t g_RxByte   = 0;                /* 接收缓冲 */

/*==============================================================================
 * 函数名：SystemClock_Config
 * 功能  ：系统时钟配置 —— 使用内部 HSI(8MHz)
 * 说明  ：串口波特率精度依赖 APB2 时钟。
 *         HSI 8MHz 下：USARTDIV = 8000000/(16×115200) = 4.3403
 *                      BRR = (4<<4)|5 = 0x45
 *                      实际波特率 = 8000000/69 ≈ 115942 bps
 *                      误差 ≈ 0.64%（远小于允许的 2%，通信正常）
 *         不用外部晶振（HSE）是因为晶振若不起振，HAL 会卡死在初始化里。
 *============================================================================*/
void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSION;                        /* 打开内部 8MHz RC */
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U)         /* 等它就绪 */
    {
        ;
    }

    RCC->CFGR &= ~RCC_CFGR_SW;                      /* 系统时钟切换到 HSI */
    RCC->CFGR |=  RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
        ;
    }

    /* 各总线不分频：HCLK = PCLK1 = PCLK2 = 8MHz（USART1 挂在 APB2）*/
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
}

/*==============================================================================
 * 函数名：Delay_ms
 * 功能  ：毫秒级延时（SysTick 轮询方式，不依赖中断）
 * 说明  ：不用 HAL_Delay() 是因为它依赖 SysTick 中断，一旦中断异常会死等。
 *         这个函数直接读 SysTick 的 COUNTFLAG 标志位，绝对安全。
 *============================================================================*/
#if (SEND_DELAY_MS > 0)
static void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        SYSTICK_LOAD = 1000U - 1U;      /* 从 999 递减到 0 = 1000 次 = 1ms */
        SYSTICK_VAL  = 0U;              /* 清零计数器和标志位 */
        SYSTICK_CTRL = 0x00000005U;     /* bit2=0 用 HCLK/8；bit0=1 启动计数 */

        while ((SYSTICK_CTRL & (1U << 16)) == 0U)
        {
            ;                           /* 等 COUNTFLAG 置 1，即 1ms 到 */
        }
        SYSTICK_CTRL = 0x00000000U;     /* 关闭 SysTick */
    }
}
#endif

/*==============================================================================
 * 函数名：Error_Handler
 * 功能  ：HAL 出错时停在这里（死循环）
 *============================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        ;
    }
}

/*==============================================================================
 * ★★★ 中断服务函数（用 HAL 库必须提供）★★★
 *============================================================================*/

/*---- SysTick 中断：每 1ms 进一次，给 HAL 的毫秒计数器加 1 ----
 * ★ 没有这个函数，程序会在开机 1ms 后卡死在 Default_Handler 里！
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/*---- 以下是异常处理函数（HAL 库内部会用到）----*/
void NMI_Handler(void)        { while (1) { ; } }
void HardFault_Handler(void)  { while (1) { ; } }
void MemManage_Handler(void)  { while (1) { ; } }
void BusFault_Handler(void)   { while (1) { ; } }
void UsageFault_Handler(void) { while (1) { ; } }
void SVC_Handler(void)        { ; }
void DebugMon_Handler(void)   { ; }
void PendSV_Handler(void)     { ; }

/*==============================================================================
 * 函数名：HAL_UART_MspInit
 * 功能  ：USART1 底层初始化（配置 GPIO 和时钟）
 * 说明  ：HAL 库的弱函数回调，HAL_UART_Init() 会自动调用它。
 *============================================================================*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        /*---- 1) 开时钟：USART1 和 PA9/PA10 都挂在 APB2 总线上 ----*/
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /*---- 2) PA9 = USART1_TX：复用推挽输出 ----*/
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*---- 3) PA10 = USART1_RX：上拉输入 ----*/
        GPIO_InitStruct.Pin  = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/*==============================================================================
 * 函数名：MX_USART1_UART_Init
 * 功能  ：初始化 USART1：115200 波特率，8 位数据，1 位停止，无校验
 *============================================================================*/
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = UART_BAUDRATE;        /* 115200 */
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;   /* 8 位数据位 */
    huart1.Init.StopBits     = UART_STOPBITS_1;      /* 1 位停止位 */
    huart1.Init.Parity       = UART_PARITY_NONE;     /* 无校验位 */
    huart1.Init.Mode         = UART_MODE_TX_RX;      /* 收、发都使能 */
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;  /* 不用硬件流控 */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; /* 16 倍过采样 */

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/*==============================================================================
 * 函数名：UART_SendString
 * 功能  ：发送一个字符串
 *============================================================================*/
static void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), UART_TIMEOUT);
}

/*==============================================================================
 * 函数名：ProcessCommand
 * 功能  ：处理上位机发来的命令
 *   '#' → 暂停发送
 *   '*' → 继续发送
 *============================================================================*/
static void ProcessCommand(uint8_t ch)
{
    if (ch == '#')
    {
        g_SendFlag = 0;                     /* 暂停发送 */
        UART_SendString("\r\n[STM32] got '#' -> PAUSED\r\n");
    }
    else if (ch == '*')
    {
        g_SendFlag = 1;                     /* 继续发送 */
        UART_SendString("\r\n[STM32] got '*' -> RESUMED\r\n");
    }
}

/*==============================================================================
 * 函数名：main
 *============================================================================*/
int main(void)
{
    HAL_Init();                     /* HAL 库初始化（会打开 SysTick 中断）*/
    SystemClock_Config();           /* 系统时钟：内部 HSI 8MHz */
    MX_USART1_UART_Init();          /* 初始化 USART1 */

    /* 开机提示信息 */
    UART_SendString("\r\n===== STM32 USART1 DEMO (HAL) =====\r\n");
    UART_SendString("USART1  115200bps  8 data bits  1 stop bit  no parity\r\n");
    UART_SendString("Send '#' to PAUSE, send '*' to RESUME\r\n\r\n");

    while (1)
    {
        /*---- ① 轮询方式接收命令（超时 10ms）----*/
        if (HAL_UART_Receive(&huart1, &g_RxByte, 1, 10U) == HAL_OK)
        {
            ProcessCommand(g_RxByte);
        }

        /*---- ② 允许发送时才发送 ----*/
        if (g_SendFlag != 0U)
        {
            UART_SendString("hello windows!\r\n");
        }

#if (SEND_DELAY_MS > 0)
        Delay_ms(SEND_DELAY_MS);
#endif
    }
}
