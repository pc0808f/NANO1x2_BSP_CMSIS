/**
 ******************************************************************************
 * @file      startup_nano1xx
 * @author    Coocox
 * @version   V1.1
 * @date      04/15/2014
 * @brief     NANO1xx Devices Startup code.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the vector table entries with the exceptions ISR address
 *                - Initialize data and bss
 *                - Setup the microcontroller system.
 *                - Call the application's entry point.
 *            After Reset the Cortex-M0 processor is in Thread mode,
 *            priority is Privileged, and the Stack is set to Main.
 *******************************************************************************
 */
 
 
 /*----------Stack Configuration----------------------------------------------*/
#define STACK_SIZE       0x00000100      /*!< The Stack size suggest using even number    */
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];


/*----------Macro definition--------------------------------------------------*/
#define WEAK __attribute__ ((weak))


/*----------Declaration of the default fault handlers-------------------------*/
/* System exception vector handler */
__attribute__ ((used))
void WEAK  Reset_Handler(void);
void WEAK  NMI_Handler(void);
void WEAK  HardFault_Handler(void);
void WEAK  SVC_Handler(void);
void WEAK  PendSV_Handler(void);
void WEAK  SysTick_Handler(void);

void WEAK  BOD_IRQHandler(void);
void WEAK  WDT_IRQHandler(void);
void WEAK  EINT0_IRQHandler(void);
void WEAK  EINT1_IRQHandler(void);
void WEAK  GPABC_IRQHandler(void);
void WEAK  GPDEF_IRQHandler(void);
void WEAK  PWM0_IRQHandler(void);
void WEAK  PWM1_IRQHandler(void);
void WEAK  TMR0_IRQHandler(void);
void WEAK  TMR1_IRQHandler(void);
void WEAK  TMR2_IRQHandler(void);
void WEAK  TMR3_IRQHandler(void);
void WEAK  UART0_IRQHandler(void);
void WEAK  UART1_IRQHandler(void);
void WEAK  SPI0_IRQHandler(void);
void WEAK  SPI1_IRQHandler(void);
void WEAK  HIRC_IRQHandler(void);
void WEAK  I2C0_IRQHandler(void);
void WEAK  I2C1_IRQHandler(void);
void WEAK  SC0_IRQHandler(void);
void WEAK  SC1_IRQHandler(void);
void WEAK  LCD_IRQHandler(void);
void WEAK  PDMA_IRQHandler(void);
void WEAK  PDWU_IRQHandler(void);
void WEAK  ADC_IRQHandler(void);
void WEAK  ACMP_IRQHandler(void);
void WEAK  RTC_IRQHandler(void);


/*----------Symbols defined in linker script----------------------------------*/
extern unsigned long _sidata;    /*!< Start address for the initialization
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */
extern unsigned long _edata;     /*!< End address for the .data section       */
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */
extern void _eram;               /*!< End address for ram                     */


/*----------Function prototypes-----------------------------------------------*/
extern int main(void);           /*!< The entry point for the application.    */
extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
void Default_Reset_Handler(void);   /*!< Default reset handler                */
static void Default_Handler(void);  /*!< Default exception handler            */


/**
  *@brief The minimal vector table for a Cortex M3. Note that the proper constructs
  *       must be placed on this to ensure that it ends up at physical address
  *       0x00000000.
  */
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
  /*----------Core Exceptions------------------------------------------------ */
  (void *)&pulStack[STACK_SIZE],     /*!< The initial stack pointer         */
  Reset_Handler,             /*!< Reset Handler                               */
  NMI_Handler,               /*!< NMI Handler                                 */                    
  HardFault_Handler,
  0,0,0,0,0,0,0,
  SVC_Handler,               /*!< SVCall Handler                              */
  0,0,
  PendSV_Handler,            /*!< PendSV Handler                              */
  SysTick_Handler,           /*!< SysTick Handler                             */

  /*----------External Exceptions---------------------------------------------*/
  BOD_IRQHandler,            /*!<  0: Brownout low voltage detected interrupt                             */
  WDT_IRQHandler,            /*!<  1: Watch Dog Timer interrupt                                           */
  EINT0_IRQHandler,          /*!<  2: External signal interrupt from PB.14 pin                            */
  EINT1_IRQHandler,          /*!<  3: External signal interrupt from PB.15 pin                            */
  GPABC_IRQHandler,          /*!<  4: External signal interrupt from PA[15:0]/PB[13:0]/PC[15:0]           */
  GPDEF_IRQHandler,          /*!<  5: External signal interrupt from PD[15:0]/PE[13:0]/PF[5:0]            */
  PWM0_IRQHandler,           /*!<  6: PWM 0 interrupt                                                     */
  PWM1_IRQHandler,           /*!<  7: PWM 1 interrupt                                                     */
  TMR0_IRQHandler,           /*!<  8: Timer 0 interrupt                                                   */
  TMR1_IRQHandler,           /*!<  9: Timer 1 interrupt                                                   */
  TMR2_IRQHandler,           /*!<  10: Timer 2 interrupt                                                  */
  TMR3_IRQHandler,           /*!<  11: Timer 3 interrupt                                                  */
  UART0_IRQHandler,          /*!<  12: UART 0 interrupt                                                   */
  UART1_IRQHandler,          /*!<  13: UART 1 interrupt                                                   */
  SPI0_IRQHandler,           /*!<  14: SPI 0 interrupt                                                    */
  SPI1_IRQHandler,           /*!<  15: SPI 1 interrupt                                                    */                                                   
  HIRC_IRQHandler,           /*!<  17: IRC TRIM interrupt                                                 */
  I2C0_IRQHandler,           /*!<  18: I2C 0 interrupt                                                    */
  I2C1_IRQHandler,           /*!<  19: I2C 1 interrupt                                                    */
  Default_Handler,           /*!<  20: Reserved                                                           */
  SC0_IRQHandler,            /*!<  21: Smart Card 0 interrupt                                             */
  SC1_IRQHandler,            /*!<  22: Smart Card 1 interrupt                                             */
  ACMP_IRQHandler,
  LCD_IRQHandler,            /*!<  25: LCD interrupt                                                      */
  PDMA_IRQHandler,           /*!<  26: DMA interrupt                                                      */
  PDWU_IRQHandler,           /*!<  28: Clock control interrupt for chip wake-up from power-down state     */
  ADC_IRQHandler,            /*!<  29: ADC interrupt                                                      */
  RTC_IRQHandler,            /*!<  31: Real time clock interrupt                                          */
};



/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called.
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  /* Initialize data and bss */
  unsigned long *pulSrc, *pulDest;

  /* Copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }

  /* Zero fill the bss segment. */ 
  for(pulDest = &_sbss; pulDest < &_ebss; )
  {
    *(pulDest++) = 0;
  }
  
  /* Setup the microcontroller system. */
  //SystemInit();

  __asm("  LDR     R0, =0x50000100\n"
        "  LDR     R1, =0x59\n"
        "  STR     R1, [R0]\n"
        "  LDR     R1, =0x16\n"
        "  STR     R1, [R0]\n"
        "  LDR     R1, =0x88\n"
        "  STR     R1, [R0]\n"
        "  LDR     R2, =0x50000060\n"
	    "  LDR     R1, =0x00005AA5\n"
		"  STR     R1, [R2]\n"
		"  MOVS    R1, #0\n"
        "  STR     R1, [R0]");

  /* Call the application's entry point.*/
  main();
}


/**
  *@brief Provide weak aliases for each Exception handler to the Default_Handler.
  *       As they are weak aliases, any function with the same name will override
  *       this definition.
  */
#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler
#pragma weak HardFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler

#pragma weak BOD_IRQHandler = Default_Handler
#pragma weak WDT_IRQHandler = Default_Handler
#pragma weak EINT0_IRQHandler = Default_Handler
#pragma weak EINT1_IRQHandler = Default_Handler
#pragma weak GPABC_IRQHandler = Default_Handler
#pragma weak GPDEF_IRQHandler = Default_Handler
#pragma weak PWM0_IRQHandler = Default_Handler
#pragma weak PWM1_IRQHandler = Default_Handler
#pragma weak TMR0_IRQHandler = Default_Handler
#pragma weak TMR1_IRQHandler = Default_Handler
#pragma weak TMR2_IRQHandler = Default_Handler
#pragma weak TMR3_IRQHandler = Default_Handler
#pragma weak UART0_IRQHandler = Default_Handler
#pragma weak UART1_IRQHandler = Default_Handler
#pragma weak SPI0_IRQHandler = Default_Handler
#pragma weak SPI1_IRQHandler = Default_Handler
#pragma weak HIRC_IRQHandler = Default_Handler
#pragma weak I2C0_IRQHandler = Default_Handler
#pragma weak I2C1_IRQHandler = Default_Handler
#pragma weak SC0_IRQHandler = Default_Handler
#pragma weak SC1_IRQHandler = Default_Handler
#pragma weak LCD_IRQHandler = Default_Handler
#pragma weak PDMA_IRQHandler = Default_Handler
#pragma weak PDWU_IRQHandler = Default_Handler
#pragma weak ADC_IRQHandler = Default_Handler
#pragma weak ACMP_IRQHandler = Default_Handler
#pragma weak RTC_IRQHandler = Default_Handler


/**
  * @brief  This is the code that gets called when the processor receives an
  *         unexpected interrupt.  This simply enters an infinite loop,
  *         preserving the system state for examination by a debugger.
  * @param  None
  * @retval None
  */
static void Default_Handler(void)
{
	/* Go into an infinite loop. */
	while (1)
	{
	}
}

/*********************** (C) COPYRIGHT 2014 Coocox ************END OF FILE*****/
