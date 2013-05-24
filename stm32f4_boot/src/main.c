/**
 ******************************************************************************
 * @file       main.c
 * @version    V0.0.1
 * @brief      mx207��boot���򣬱�����ʵ������Ӧ�ó����Լ���������Ӧ�ó�����
 * @details
 * @copyright
 *
 ******************************************************************************
 */
/*-----------------------------------------------------------------------------
 Section: Includes
 ----------------------------------------------------------------------------*/
#include <types.h>
#include <target.h>
#include <update.h>
#include <cfg.h>
#include <stm32f407.h>
/*----------------------------------------------------------------------------
 Section: Type Definitions
 ----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 Section: Private Function Prototypes
 ----------------------------------------------------------------------------*/
static void reset_routine(void);
static void default_routine(void);
static void dummy(void);
static void systick_routine(void);
int main(void);

/*-----------------------------------------------------------------------------
 Section: Globals
 ----------------------------------------------------------------------------*/
extern uint32_t _etext; //The end of text;
extern uint32_t _bss;   //The start of bss;
extern uint32_t _ebss;
extern uint32_t _data;
extern uint32_t _edata;

extern void set_led(uint8_t on);

/*-----------------------------------------------------------------------------
 Section: Private Variables
 ----------------------------------------------------------------------------*/
static uint32_t the_run_time = 0u; /* �������������ۼ�ʱ��(��) */
__attribute__((section(".stackarea")))
static uint32_t the_cstack[STACK_SIZE];

__attribute__((section(".isr_vector")))
const VOIDFUNCPTR vector_table[] =
{
    (VOIDFUNCPTR)((uint32_t)the_cstack + sizeof(the_cstack)),/* ջ��ָ�� */
    reset_routine,      /* ��λ�ж�  */
    default_routine,    /* NMI�ж� */
    default_routine,    /* Ӳ������ */
    default_routine,    /* �洢���������� */
    default_routine,    /* ���ߴ��� */
    default_routine,    /* �÷����� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    systick_routine, /* 15ϵͳ��ʱ�� */

    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 21���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 31���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 41���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 51���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 61���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 71���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */
    dummy, /* ���� */

    dummy, /* 81���� */
    dummy, /* 82���� */
    dummy, /* 83 */
};

/*-----------------------------------------------------------------------------
 Section: Function Definitions
 ----------------------------------------------------------------------------*/
/**
 ******************************************************************************
 * @brief   ��λ�ж����
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
static void
reset_routine(void)
{
    uint32_t* pul_src;
    uint32_t* pul_dest;

    /* �����渴�Ƴ�ʼ�����ݶε�SRAM��. */
    for (pul_dest = &_data, pul_src = &_etext; pul_dest < &_edata;)
    {
        *pul_dest++ = *pul_src++;
    }

    /* bss������ */
    for (pul_dest = &_bss; pul_dest < &_ebss;)
    {
        *pul_dest++ = 0;
    }
    /* ��ת�������� */
    main();
}

/**
 ******************************************************************************
 * @brief   �쳣�ж����
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
static void
default_routine(void)
{
    while (TRUE)
    {
    }
}

/**
 ******************************************************************************
 * @brief   �����ж����
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
static void
dummy(void)
{
    return;
}

/**
 ******************************************************************************
 * @brief   ��ʱ���жϴ���
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 * @note
 *  ע�⣺Ҫ��ÿ�����ж�һ��
 ******************************************************************************
 */
static void
systick_routine(void)
{
    static uint32_t ms = 0u; /* ������ */
    static uint8_t led = 0u;

    if (ms % (get_run_status() == RS_IDLE ? 500 : 100) == 0)
    {
        set_led(led++);
    }
    /* ÿ�����ж�һ�� */
    ms++;
    if (ms < 1000)
    {
        return;
    }
    ms = 0;

    /* ���������������ʱ��(5����)������ι����ʹ���Զ���λ */
    if (the_run_time >= (5u * 60u))
    {
        /* reboot */
        print("reboot!\r\n");
        NVIC_SystemReset();
        return;
    }
    the_run_time++;
}

/**
 ******************************************************************************
 * @brief   ����������
 * @param[in]  None
 * @param[out] None
 *
 * @retval     None
 ******************************************************************************
 */
int32_t
main(void)
{
    pFunction Jump_To_Application;
    uint32_t JumpAddress;

    /* ��ʼ���豸 */
    hw_init();

    print("\r\r\r\r\n");
    print("*********************************************\r\n");
    print("*               BootLoader v1.1             *\r\n");
    print("*         CopyRight 2013 discovery407.      *\r\n");
    print("*********************************************\r\n");

    /* ������ʱ�� */
    systick_open();

    /* �������� */
    if (OK != uart_update())
    {
        return -1;
    }

    /* �رն�ʱ�� */
    systick_stop();
    /* ��ת��Ӧ��ϵͳ */
    print("Enter application ...\r\n");

    /* Jump to user application */
    JumpAddress = *(__IO uint32_t*) (APP_START_ADDRESS + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
//    __set_MSP(*(__IO uint32_t*) APP_START_ADDRESS);
    Jump_To_Application();

    return 0;
}

/**
 ******************************************************************************
 * @brief   ��ȡ��ǰϵͳ����ʱ��
 * @param[in]  None
 * @param[out] None
 *
 * @retval  ���е�ʱ��
 ******************************************************************************
 */
extern uint32_t
get_systime(void)
{
    return the_run_time;
}

/*--------------------------------End of main.c------------------------------*/
