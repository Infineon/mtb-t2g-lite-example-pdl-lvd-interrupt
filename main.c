/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the T2G LVD Interrupt Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* led blink times */
#define LEDCYCLE          5

/*******************************************************************************
* Global Variables
********************************************************************************/
/* LVD_1 interrupt configuration structure */
cy_stc_sysint_t LVD_1_IRQ_cfg  = {
    /* Bit 0-15 of intrSrc is used to store system interrupt value and bit 16-31 to store CPU IRQ value */
    .intrSrc = ((NvicMux3_IRQn << CY_SYSINT_INTRSRC_MUXIRQ_SHIFT) | srss_interrupt_IRQn),
    /* Interrupt priority is 2 */
    .intrPriority = NvicMux3_IRQn 
 };

/* LVD interrupt flag */
bool flagLVDIRQ = false;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void ISR_LVD_1(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* The main function performs the following actions:
*  1. Configures LVD component.
*  2. Set LVD parameter.
*  3. Set LVD interrupt.
*  4. Adjust the VDDD voltage below 2.8V and watch LED2, if LVD interrupt was generated,
*     the LED2 status was changed.
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Disable the LVDs */
    Cy_LVD_HT_Disable(CY_LVD_HT_SELECT_LVD_1);

    /* Disable LVD_1 interrupt */
    Cy_LVD_HT_ClearInterruptMask(CY_LVD_HT_SELECT_LVD_1);

    /* Configure the threshold value, set LVD_1 threshold is 2.8V */
    Cy_LVD_HT_SetThreshold(CY_LVD_HT_SELECT_LVD_1, CY_LVD_HT_THRESHOLD_2_8_V); 

    /* Set LVD_1 generate interrupt */
    Cy_LVD_HT_SetActionConfig(CY_LVD_HT_SELECT_LVD_1, CY_LVD_HT_ACTION_INTERRUPT);

    /* Set which edge(s) will trigger an interrupt or fault for LVDs, 
       select falling edge for LVD_1, when VDDD below threshold LVD will generate interrupt */
    Cy_LVD_HT_SetInterruptConfig(CY_LVD_HT_SELECT_LVD_1,CY_LVD_HT_INTR_FALLING); 

    /* Enable LVDs */
    Cy_LVD_HT_Enable(CY_LVD_HT_SELECT_LVD_1);

    /* Wait for at least 25us to get LVD stabilized (reserved, wait for the resolution from SW team) */
    Cy_SysLib_DelayUs(25U);

    /* Clear LVD_1 interrupt */
    Cy_LVD_HT_ClearInterrupt(CY_LVD_HT_SELECT_LVD_1);

    /* Enable LVD_1 interrupts */
    Cy_LVD_HT_SetInterruptMask(CY_LVD_HT_SELECT_LVD_1);

    /* Sets up the interrupt handler */
    Cy_SysInt_Init(&LVD_1_IRQ_cfg, ISR_LVD_1);

    /* Enable the LVD interrupt in NVIC */
    NVIC_EnableIRQ((IRQn_Type) NvicMux3_IRQn);

    /* Enable LVDs work in deep sleep mode */
    Cy_LVD_HT_DeepSleepEnable(CY_LVD_HT_SELECT_LVD_1);

    for (;;)
    {
        /* Blink LED2 after LVD generate interrupt */
        if(flagLVDIRQ)
        {
            flagLVDIRQ = 0;

            for(int i=0; i<LEDCYCLE*2; i++)
            {
                CyDelay(500u);
                cyhal_gpio_toggle(CYBSP_USER_LED2);
            }
        }

        /* Normal running */
        CyDelay(500u);
        cyhal_gpio_toggle(CYBSP_USER_LED1);
    }
}

/*******************************************************************************
* Function Name: void ISR_LVD_1(void)
********************************************************************************
* Summary:
*  LVD_1 interrupt handler .
*
*******************************************************************************/
void ISR_LVD_1(void)
{
    /* Judge which LVD cause this interrupt */
    if( SRSS_SRSS_INTR_MASK_HVLVD1_Msk == Cy_LVD_HT_GetInterruptStatusMasked(CY_LVD_HT_SELECT_LVD_1)) 
    {
        /* Set LVD interrupt flag */
        flagLVDIRQ = true;

        /* Clear LVD_1 interrupt */
        Cy_LVD_HT_ClearInterrupt(CY_LVD_HT_SELECT_LVD_1);
    }
}

/* [] END OF FILE */
