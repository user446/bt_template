/* slld_hal_example.c - SLLD Hardware Abstraction Layer example Code */

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

// ***************************************************************************
//  FLASH_READ - HAL read function
//
//  input : device_num            device number to which operation will be done
//          command               write a single command byte to flash
//          sys_addr              system address to be used
//          data_buffer           Pointer to the data buffer where to store the read data
//          Number_Of_Read_Bytes  number of bytes to be read
//
//  return value : status of the operation - FAIL or SUCCESS
// ***************************************************************************
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
    int data_cycle, Number_Of_Dummy_Bytes = 0;

    // Select the device
    PISMOLED &= 0xFE; // LED1 ON
    SPICS = !device_num;

    // Write the command
    SPIDATA = command;

    // Write the address
    if (sys_addr != ADDRESS_NOT_USED)
    {
        switch (command)
        {
        case SPI_DUALIO_HPRD_CMD:
        {
            SPIDATA_D = (BYTE)((sys_addr >> 16) & 0x000000FF);
            SPIDATA_D = (BYTE)((sys_addr >>  8) & 0x000000FF);
            SPIDATA_D = (BYTE)(sys_addr        & 0x000000FF);
            break;
        }
        case SPI_DUALIO_HPRD_4B_CMD:
        {
            SPIDATA_D = (BYTE)((sys_addr >> 24) & 0x000000FF);
            SPIDATA_D = (BYTE)((sys_addr >> 16) & 0x000000FF);
            SPIDATA_D = (BYTE)((sys_addr >> 8) & 0x000000FF);
            SPIDATA_D = (BYTE)(sys_addr        & 0x000000FF);
            break;
        }
        case SPI_QUADIO_HPRD_CMD:
        {
            SPIDATA_Q = (BYTE)((sys_addr >> 16) & 0x000000FF);
            SPIDATA_Q = (BYTE)((sys_addr >>  8) & 0x000000FF);
            SPIDATA_Q = (BYTE) (sys_addr        & 0x000000FF);
            break;
        }
        case SPI_QUADIO_HPRD_4B_CMD:
        {
            SPIDATA_Q = (BYTE)((sys_addr >> 24) & 0x000000FF);
            SPIDATA_Q = (BYTE)((sys_addr >> 16) & 0x000000FF);
            SPIDATA_Q = (BYTE)((sys_addr >> 8)  & 0x000000FF);
            SPIDATA_Q = (BYTE) (sys_addr        & 0x000000FF);
            break;
        }
        default:
        {
            SPIDATA = (BYTE)((sys_addr >> 16) & 0x000000FF);
            SPIDATA = (BYTE)((sys_addr >>  8) & 0x000000FF);
            SPIDATA = (BYTE) (sys_addr        & 0x000000FF);
            break;
        }
        }
    }

    // Write the dummy bytes
    switch (command)
    {
    case SPI_FAST_READ_CMD:
    case SPI_FAST_READ_4B_CMD:
    case SPI_DUALIO_RD_CMD:
    case SPI_DUALIO_RD_4B_CMD:
    case SPI_QUADIO_RD_CMD:
    case SPI_QUADIO_RD_4B_CMD:
    case SPI_OTPR_CMD:
    case SPI_READ_SFDP_CMD:
    {
        Number_Of_Dummy_Bytes = 1;
        // Write a dummy byte to the data bus
        for (data_cycle = 0; data_cycle < Number_Of_Dummy_Bytes; data_cycle++)
            SPIDATA = 0x0;
        break;
    }
    case SPI_DUALIO_HPRD_CMD:
    case SPI_DUALIO_HPRD_4B_CMD:
    {
        Number_Of_Dummy_Bytes = 1;
        // Write a dummy byte to the data bus - This is actually the mode bit
        for (data_cycle = 0; data_cycle < Number_Of_Dummy_Bytes; data_cycle++)
            SPIDATA_D = 0x0;
        break;
    }
    case SPI_QUADIO_HPRD_CMD:
    case SPI_QUADIO_HPRD_4B_CMD:
    {
        Number_Of_Dummy_Bytes = 3;
        // Write the dummy bytes to the data bus - The first byte is actually the mode bit
        for (data_cycle = 0; data_cycle < Number_Of_Dummy_Bytes; data_cycle++)
            SPIDATA_Q = 0x0;
        break;
    }
    case SPI_RES_CMD:
    {
        Number_Of_Dummy_Bytes = 3;
        // Write the dummy bytes to the data bus
        for (data_cycle = 0; data_cycle < Number_Of_Dummy_Bytes; data_cycle++)
            SPIDATA = 0x0;
        break;
    }
    default:
    {
        Number_Of_Dummy_Bytes = 0;
        break;
    }
    }

    // Read the data
    if (Number_Of_Read_Bytes != 0)
    {
        switch (command)
        {
        case SPI_DUALIO_RD_CMD:
        case SPI_DUALIO_RD_4B_CMD:
        case SPI_DUALIO_HPRD_CMD:
        case SPI_DUALIO_HPRD_4B_CMD:
        {
            // Read the data using the relevant mode
            for (data_cycle = 0; data_cycle < Number_Of_Read_Bytes; data_cycle++)
                *(data_buffer + data_cycle) = SPIDATA_D;
            break;
        }
        case SPI_QUADIO_RD_CMD:
        case SPI_QUADIO_RD_4B_CMD:
        case SPI_QUADIO_HPRD_CMD:
        case SPI_QUADIO_HPRD_4B_CMD:
        {
            // Read the data using the relevant mode
            for (data_cycle = 0; data_cycle < Number_Of_Read_Bytes; data_cycle++)
                *(data_buffer + data_cycle) = SPIDATA_Q;
            break;
        }
        default:
        {
            // Read the data using the relevant mode
            for (data_cycle = 0; data_cycle < Number_Of_Read_Bytes; data_cycle++)
                *(data_buffer + data_cycle) = SPIDATA;
            break;
        }
        }
    }

    // Deselect the device
    PISMOLED |= 0x01; // LED1 OFF
    SPICS = 0xFF;

    return(status);
}


// ***************************************************************************
//  FLASH_WRITE - HAL write function
//
//  input : device_num               device number to which operation will be done
//          command                  write a single command byte to flash
//          sys_addr                 system address to be used
//          data_buffer              Pointer to the data buffer where to store the written data
//          Number_Of_Written_Bytes  number of bytes to be written
//
//  return value : status of the operation - FAIL or SUCCESS
// ***************************************************************************
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
    int data_cycle;

    // Select the device
    PISMOLED &= 0xFE; // LED1 ON
    SPICS = !device_num;

    // Write the command
    SPIDATA = command;

    // Write the address
    if (sys_addr != ADDRESS_NOT_USED)
    {
        SPIDATA = (BYTE)((sys_addr >> 16) & 0x000000FF);
        SPIDATA = (BYTE)((sys_addr >>  8) & 0x000000FF);
        SPIDATA = (BYTE) (sys_addr        & 0x000000FF);
    }

    // Write the data
    if (Number_Of_Written_Bytes != 0)
    {
        switch (command)
        {
        case SPI_QPP_CMD:
        case SPI_QPP_4B_CMD:
        {
            // Write the data using the relevant mode
            for (data_cycle = 0; data_cycle < Number_Of_Written_Bytes; data_cycle++)
                SPIDATA_Q = *(data_buffer + data_cycle);
            break;
        }
        default:
        {
            // Write the data using the relevant mode
            for (data_cycle = 0; data_cycle < Number_Of_Written_Bytes; data_cycle++)
                SPIDATA = *(data_buffer + data_cycle);
            break;
        }
        }
    }

    // Deselect the device
    PISMOLED |= 0x01; // LED1 OFF
    SPICS = 0xFF;

    return(status);
}


/*****************************************************************************/
