/**
 ******************************************************************************
 * @file       target.c
 * @version    V0.0.1
 * @brief      �����ṩboot�йص�Ӳ����������
 * @details
 * @copyright
 *
 ******************************************************************************
 */
/*-----------------------------------------------------------------------------
 Section: Includes
 ----------------------------------------------------------------------------*/
#include <types.h>
#include <stm32f407.h>
#include <cfg.h>
/*----------------------------------------------------------------------------
 Section: Type Definitions
 ----------------------------------------------------------------------------*/
#ifndef BAUDRATE
#define BAUDRATE              115200u
#endif

#define UART_BASE             ((USART_TypeDef *) USART6_BASE)

/*-----------------------------------------------------------------------------
 Section: Private Function Prototypes
 ----------------------------------------------------------------------------*/
static void uart_init(void);
static void led_init(void);

/*-----------------------------------------------------------------------------
 Section: Function Definitions
 ----------------------------------------------------------------------------*/
/**
 ******************************************************************************
 * @brief   Ӳ����ʼ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 *
 * @note
 *  ��Ҫ�����������:
 *      1������ϵͳʱ��
 *      2����ʼ��uart��spi��ι��
 ******************************************************************************
 */
void
hw_init(void)
{
    SystemInit();

    /* ʹ������ */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
//    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    led_init();

    /* ����UART */
    uart_init();
}

/**
 ******************************************************************************
 * @brief   ����ϵͳ��ʱ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 *
 * @note
 *  ע�⣺��ʱ��������Ϊ1ms�ж�1��
 ******************************************************************************
 */
void
systick_open(void)
{
	RCC_ClocksTypeDef clocks_status;

	RCC_GetClocksFreq(&clocks_status);

    SysTick_Config(clocks_status.HCLK_Frequency / 1000);
    return;
}

/**
 ******************************************************************************
 * @brief   �ر�ϵͳ��ʱ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void
systick_stop(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 ******************************************************************************
 * @brief   uart��ʼ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void
uart_init(void)
{
#if 0
    GPIO_InitTypeDef gpio_inits;
    USART_InitTypeDef usart_inits;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //����USART1ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //����GPIOAʱ��
    /* UART IO�������� */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);//���൱��M3�Ŀ�������ʱ�ӣ�ֻ���ø��õ����ţ�
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);//

    GPIO_Init(GPIOA, &gpio_inits);
//    gpio_inits.GPIO_OType = GPIO_OType_PP;
//    gpio_inits.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_inits.GPIO_Mode = GPIO_Mode_AF;
    gpio_inits.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART Tx as alternate function  */
    gpio_inits.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &gpio_inits);

    /* Configure USART Rx as alternate function  */
    gpio_inits.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio_inits);

    usart_inits.USART_BaudRate = BAUDRATE;
    usart_inits.USART_WordLength = USART_WordLength_8b;
    usart_inits.USART_StopBits = USART_StopBits_1;
    usart_inits.USART_Parity = USART_Parity_No;
    usart_inits.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_inits.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(UART_BASE, &usart_inits);
    UART_BASE->CR1 |= USART_CR1_UE;
#endif
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //����USART6ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  //����GPIOCʱ��
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);//���൱��M3�Ŀ�������ʱ�ӣ�ֻ���ø��õ����ţ�
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);//
    /*����GPIOC*/
    GPIO_StructInit(&GPIO_InitStructure);      //ȱʡֵ����

    /*����GPIOC_Pin6ΪTX���*/
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;     //����Ϊ���ã�����ΪAF��OUT����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    /*����GPIOC_Pin7ΪRX����*/
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;     //��Ҳ����Ϊ���ã���M3��ͬ��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    /*IO���Ÿ��ù������ã���֮ǰ�汾��ͬ*/
    /*����USART6*/
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART6, &USART_InitStructure);
    USART_ClockStructInit(&USART_ClockInitStruct);    //֮ǰû������ȱʡֵ���ǲ��е�
    USART_ClockInit(USART6, &USART_ClockInitStruct);

//    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);    //ʹ�� USART6�ж�
    USART_Cmd(USART6, ENABLE);         //ʹ�� USART6
}

/**
 ******************************************************************************
 * @brief   LED��ʼ��Ϊȫ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void led_init(void)
{
	GPIO_InitTypeDef led_inits;

	led_inits.GPIO_OType = GPIO_OType_PP;
	led_inits.GPIO_PuPd = GPIO_PuPd_DOWN;
	led_inits.GPIO_Mode = GPIO_Mode_OUT;
	led_inits.GPIO_Speed = GPIO_Speed_50MHz;
	led_inits.GPIO_Pin = GPIO_Pin_12;

    GPIO_Init(GPIOD, &led_inits);
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
}

/**
 ******************************************************************************
 * @brief   ����led
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void set_led(uint8_t on)
{
    if (on & 0x01)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
    }
    else
    {
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    }
}

/**
 ******************************************************************************
 * @brief   �����ڷ���һ���ֽ�
 * @param[in]  c    : �������ֽ�
 * @param[out] None
 * @retval     None
 ******************************************************************************
 */
void
uart_send(const uint8_t c)
{
    USART_SendData(UART_BASE, (uint16_t)c);
    /* �ȴ���ɷ��� */
    while (USART_GetFlagStatus(UART_BASE, USART_FLAG_TXE) == RESET)
    {
    }
}

/**
 ******************************************************************************
 * @brief   ���ڳ��Խ���һ���ַ�
 * @param[out] *pc  : ���յ�������
 * @retval  TRUE    : ������
 * @retval  FALSE   : ������
 *
 * @details
 *
 * @note
 *
 ******************************************************************************
 */
bool_e
uart_try_receive(uint8_t *pc)
{
    if (USART_GetFlagStatus(UART_BASE, USART_FLAG_RXNE) != RESET)
    {
        *pc = (uint16_t) (UART_BASE->DR & (uint16_t) 0x01FF);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 ******************************************************************************
 * @brief   ���ڽ���ָ������������
 * @param[out] *pdata   : ���ջ���
 * @param[in]  size     : �����ճ���
 *
 * @retval     None
 ******************************************************************************
 */
void
uart_receive(uint8_t *pdata,
        uint32_t size)
{
    do
    {
        /* ���û�����ݣ���ȴ� */
        while (!uart_try_receive(pdata))
        {
        }
        pdata++;
        size--;
    } while (size > 0);
}

/**
 ******************************************************************************
 * @brief   ��ӡһ���ַ���
 * @param[in]  *str: ����ӡ���ݣ�ע��:������'\0'��β
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void
print(const char_t* str)
{
    while ('\0' != *str)
    {
        uart_send(*str);
        str++;
    }
}

/**
 ******************************************************************************
 * @brief   ��λ
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void
reset(void)
{
#if 0
    /* 0x05FA: VECTKEY����Կ�ף���ͬʱд��
     0x0004: ����оƬ����һ�θ�λ
     */
    *((uint32_t *) 0xE000ED0Cu) = 0x05FA0004u;
#endif
    NVIC_SystemReset();
}
/**
 ******************************************************************************
 * @brief   Ƭ��flash��ʼ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
void
iflash_init(void)
{
    /* ���� */
    FLASH_Unlock();
}

/**
 ******************************************************************************
 * @brief   ����Ƭ��Flashָ�����������
 * @param[in]  address  : ������ַ
 * @param[in]  size     : ��������
 *
 * @retval  TRUE    : �ɹ�
 * @retval  FALSE   : ʧ��
 ******************************************************************************
 */
bool_e
iflash_erase(uint32_t address,
        uint32_t size)
{
    uint32_t i;

    for (i = FLASH_Sector_1; i <= FLASH_Sector_3; i += 8u)
    {
        if (FLASH_COMPLETE != FLASH_EraseSector(i, VoltageRange_3))  //TODO !���÷���ȷ��
        {
            return FALSE;
        }
    }
    return TRUE;
}

/**
 ******************************************************************************
 * @brief   ��ȡƬ��Flashָ�����������
 * @param[out] *pdata   : ��ȡ����
 * @param[in]  address  : Ŀ���ַ
 * @param[in]  size     : ��ȡ����
 *
 * @retval  TRUE    : �ɹ�
 * @retval  FALSE   : ʧ��
 ******************************************************************************
 */
bool_e
iflash_read(uint8_t *pdata,
        uint32_t address,
        uint32_t size)
{
    for (uint32_t i = 0u; i < size; i++)
    {
        *pdata = *((uint8_t *)address);
        pdata++;
        address++;
    }
    return TRUE;
}

/**
 ******************************************************************************
 * @brief   �޸�Ƭ��Flashָ�����������
 * @param[in]  *pdata   : д������
 * @param[in]  address  : Ŀ���ַ
 * @param[in]  size     : д�볤��
 *
 * @retval  TRUE    : �ɹ�
 * @retval  FALSE   : ʧ��
 ******************************************************************************
 */
bool_e
iflash_write(const uint8_t *pdata,
        uint32_t address,
        uint32_t size)
{
    for (uint32_t i = 0u; i < size; i += 4u)
    {
        /* д��4�ֽ� */
        if (!FLASH_ProgramWord(address + i, *(const uint32_t *)(pdata + i)))
        {
            return (FALSE);
        }
        /* У��д���Ƿ�ɹ� */
        if (*(uint32_t *)(address + i) != *(const uint32_t *)(pdata + i))
        {
            return (FALSE);
        }
    }

    return (TRUE);
}

/**
 ******************************************************************************
 * @brief   �ȴ�
 * @param[in]  usec     : �ȴ�ʱ��
 *
 * @retval     None
 ******************************************************************************
 */
void
delay(const uint32_t usec)
{
    volatile uint32_t count = 0;
    const uint32_t utime = 120 * usec;
    do
    {
        if (++count > utime)
        {
            return;
        }
    } while (TRUE);
}

/*--------------------------------End of target.c----------------------------*/
