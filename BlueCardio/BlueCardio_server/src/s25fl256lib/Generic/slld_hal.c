/* slld_hal.c - Hardware Abstraction Layer Code for Cyress's SLLD */
 
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

#include <stdio.h>
#include "slld_targetspecific.h"
#include "slld.h"
#include "slld_hal.h"


//***************************************************************************
//  FLASH_READ - HAL read function
//
//  input : device_num            device number to which operation will be done
//          command               write a single command byte to flash
//          sys_addr              system address to be used
//          data_buffer           Pointer to the data buffer where to store the read data
//          Number_Of_Read_Bytes  number of bytes to be read
//
//  return value : status of the operation - FAIL or SUCCESS
//***************************************************************************
SLLD_STATUS FLASH_READ
(
BYTE    device_num,                     /* device number to which operation will be done */
BYTE     command,                        /* write a single command byte to flash */
ADDRESS  sys_addr,                       /* system address to be used */
BYTE    *data_buffer,                    /* Pointer to the data buffer containing data to be written */
int      Number_Of_Read_Bytes            /* number of bytes to be read */
)
{
    SLLD_STATUS status = SLLD_OK;
    int Number_Of_Dummy_Bytes = 0;

    // Select the device - Insert code to drive CS# low
    printf("Drive CS# low\n");
    
    // Write the command - Insert code to write command to the device
    printf("writing command: 0x%X\n", command);

    // Write the address - Insert HAL specific code to write the address to the device
    // in the selected mode based on command type
    if (sys_addr != ADDRESS_NOT_USED)
    {
        switch (command)
        {
            case SPI_DUALIO_HPRD_CMD:
            {
                // Shift out the address two bits at a time in 3-bytes addressing scheme
                printf("Dual address write in 3-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
            case SPI_DUALIO_HPRD_4B_CMD:
            {
                // Shift out the address two bits at a time in 4-bytes addressing scheme
                printf("Dual address write in 4-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
            case SPI_QUADIO_HPRD_CMD:
            {
                // Shift out the address four bits at a time in 3-bytes addressing scheme
                printf("Quad address write in 3-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
            case SPI_QUADIO_HPRD_4B_CMD:
            {
                // Shift out the address four bits at a time in 4-bytes addressing scheme
                printf("Quad address write in 4-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
            case SPI_READ_4B_CMD:
            case SPI_FAST_READ_4B_CMD:
            case SPI_DUALIO_RD_4B_CMD:
            case SPI_QUADIO_RD_4B_CMD:
            {
                // Shift out the address one bit at a time in 4-bytes addressing scheme
                printf("Single address write in 4-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
            default:
            {
                // Shift out the address one bit at a time in 3-bytes addressing scheme
                printf("Single address write in 3-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
        }
    }

    // Write the dummy bytes - Insert HAL specific code to write the dummy bytes to the device
    switch (command)
    {
        case SPI_FAST_READ_CMD:
        case SPI_FAST_READ_4B_CMD:
        case SPI_DUALIO_RD_CMD:
        case SPI_DUALIO_RD_4B_CMD:
        case SPI_QUADIO_RD_CMD:
        case SPI_QUADIO_RD_4B_CMD:
        case SPI_OTPR_CMD:
        {
            Number_Of_Dummy_Bytes = 1;
            // Write a dummy byte to the data bus
            printf("Writing a dummy byte in single mode, command: 0x%X\n", command);
            break;
        }
        case SPI_DUALIO_HPRD_CMD:
        case SPI_DUALIO_HPRD_4B_CMD:
        {
            Number_Of_Dummy_Bytes = 1;
            // Write a dummy byte to the data bus - This is actually the mode bit
            printf("Writing a dummy byte in dual mode, command: 0x%X\n", command);
            break;
        }
        case SPI_QUADIO_HPRD_CMD:
        case SPI_QUADIO_HPRD_4B_CMD:
        {
            Number_Of_Dummy_Bytes = 3;
            // Write 3 dummy bytes to the data bus - The first byte is actually the mode bit
            printf("Writing 3 dummy bytes in quad mode, command: 0x%X\n", command);
            break;
        }
        case SPI_RES_CMD:
        {
            Number_Of_Dummy_Bytes = 3;
            // Write 3 dummy bytes to the data bus
            printf("Writing 3 dummy bytes in single mode, command: 0x%X\n", command);
            break;
        }
        default:
        {
            // Default to no dummy bytes written
            Number_Of_Dummy_Bytes = 0;
            break;
        }
    }

    // Read the data - Insert HAL specific code to read data from the device
    if (Number_Of_Read_Bytes != 0)
    {
        switch (command)
        {
            case SPI_DUALIO_RD_CMD:
            case SPI_DUALIO_RD_4B_CMD:
            case SPI_DUALIO_HPRD_CMD:
            case SPI_DUALIO_HPRD_4B_CMD:
            {
                // Read the data using dual mode
                printf("Read the data using dual mode, command: 0x%X\n", command);
                break;
            }
            case SPI_QUADIO_RD_CMD:
            case SPI_QUADIO_RD_4B_CMD:
            case SPI_QUADIO_HPRD_CMD:
            case SPI_QUADIO_HPRD_4B_CMD:
            {
                // Read the data using quad mode
                printf("Read the data using quad mode, command: 0x%X\n", command);
                break;
            }
            default:
            {
                // Read the data using the single mode
                printf("Read the data using single mode, command: 0x%X\n", command);
                break;
            }
        }
    }

    // Deselect the device - Insert HAL specific code to drive CS# high
    printf("Drive CS# high\n");

    return(status);
}


//***************************************************************************
//  FLASH_WRITE - HAL write function
//
//  input : device_num               device number to which operation will be done
//          command                  write a single command byte to flash
//          sys_addr                 system address to be used
//          data_buffer              Pointer to the data buffer where to store the written data
//          Number_Of_Written_Bytes  number of bytes to be written
//
//  return value : status of the operation - FAIL or SUCCESS
//***************************************************************************
SLLD_STATUS FLASH_WRITE
(
BYTE    device_num,                     /* device number to which operation will be done */
BYTE     command,                        /* write a single command byte to flash */
ADDRESS  sys_addr,                       /* system address to be used */
BYTE    *data_buffer,                    /* Pointer to the data buffer containing data to be written */
int      Number_Of_Written_Bytes         /* number of bytes to be written */
)
{
    SLLD_STATUS status = SLLD_OK;

    // Select the device - Insert HAL specific code to drive CS# low
    printf("Drive CS# low\n");
    
    // Write the command - Insert HAL specific code to write command to the device
    printf("writing command: 0x%X\n", command);

    // Write the address - Insert HAL specific code to write the address to the device
    // in the selected mode based on command type
    if (sys_addr != ADDRESS_NOT_USED)
    {
        switch (command)
        {
            case SPI_PP_4B_CMD:
            case SPI_QPP_4B_CMD:
            case SPI_P8E_4B_CMD:
            case SPI_SE_4B_CMD:
            {
                // Shift out the address one bit at a time in 4-bytes addressing scheme
                printf("Single address write in 4-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
            default:
            {
                // Shift out the address one bit at a time in 3-bytes addressing scheme
                printf("Single address write in 3-bytes addressing scheme, address: 0x%X, command: 0x%X\n", sys_addr, command);
                break;
            }
        }
    }

    // Write the data - Insert HAL specific code to write data to the device
    if (Number_Of_Written_Bytes != 0)
    {
        switch (command)
        {
            case SPI_QPP_CMD:
            case SPI_QPP_4B_CMD:
            {
                // Write the data using quad mode
                printf("Write the data using quad mode\n");
                break;
            }
            default:
            {
                // Write the data using single mode
                printf("Write the data using single mode\n");
                break;
            }
        }
    }

    // Deselect the device - Insert HAL specific code to drive CS# high
    printf("Drive CS# high\n");

    return(status);
}


/*****************************************************************************/
