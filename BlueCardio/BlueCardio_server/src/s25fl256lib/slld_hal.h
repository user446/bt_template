/* slld_hal.h - HAL Header file for Cypress SPI-flash Low Level Driver */
 
/**************************************************************************
 * 2003-2016, Cypress Semiconductor Corporation or a subsidiary of Cypress 
 * Semiconductor Corporation.  All rights reserved.
 * 
 * This software, including source code, documentation and related materials 
 * (Software), is owned by Cypress Semiconductor Corporation or one of its 
 * subsidiaries (Cypress) and is protected by and subject to worldwide patent 
 * protection (United States and foreign), United States copyright laws and 
 * international treaty provisions. Therefore, you may use this Software only 
 * as provided in the license agreement accompanying the software package from 
 * which you obtained this Software (EULA).
 * 
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, 
 * non-transferable license to copy, modify, and compile the Software source code 
 * solely for use in connection with Cypress's integrated circuit products.  Any 
 * reproduction, modification, translation, compilation, or representation of 
 * this Software except as specified above is prohibited without the express 
 * written permission of Cypress.
 * 
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
 * reserves the right to make changes to the Software without notice. Cypress does 
 * not assume any liability arising out of the application or use of the Software 
 * or any product or circuit described in the Software. Cypress does not authorize 
 * its products for use in any products where a malfunction or failure of the Cypress 
 * product may reasonably be expected to result in significant property damage, injury 
 * or death (High Risk Product). By including Cypress's product in a High Risk Product, 
 * the manufacturer of such system or application assumes all risk of such use and in 
 * doing so agrees to indemnify Cypress against all liability.
 */

#ifndef __INC_lld_halh
#define __INC_lld_halh

// Users should define what controller has been used for their customerized 
// enviroment. The following code is just an implementation example, which should
// be modified by the user accordingly to match their specific hardware environment.  

#ifdef USER_CONTROLLER   
extern unsigned long base_addr_g;
#define CONTROLLER_BUFFER_SIZE   1024
#define SPIADDR   ((volatile unsigned char *)base_addr_g)
#define SPIDATA   (*SPIADDR)
#define SPIDATA_D (*(SPIADDR + 0x01))
#define SPIDATA_Q (*(SPIADDR + 0x02))
#define SPIHOLD   (*(SPIADDR + 0x04))
#define SPICS     (*(SPIADDR + 0x10))
#define SPIWP     (*(SPIADDR + 0x30))
#define PISMOLED  (*(SPIADDR + 0x40))
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* public function prototypes */

// HAL read functions API
SLLD_STATUS FLASH_READ
(
BYTE    device_num,                                /* device number to which operation will be done */
BYTE    command,                                    /* write a single command byte to flash */
ADDRESS sys_addr,                                   /* system address to be used */
BYTE   *data_buffer,                                /* Pointer to the data buffer where to store the read data */
int     Number_Of_Read_Bytes                        /* number of bytes to be read */
);

// HAL write functions API
SLLD_STATUS FLASH_WRITE
(
BYTE    device_num,                                /* device number to which operation will be done */
BYTE    command,                                    /* write a single command byte to flash */
ADDRESS sys_addr,                                   /* system address to be used */
BYTE   *data_buffer,                                /* Pointer to the data buffer containing data to be written */
int     Number_Of_Written_Bytes                     /* number of bytes to be written */
);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __INC_lld_halh */

