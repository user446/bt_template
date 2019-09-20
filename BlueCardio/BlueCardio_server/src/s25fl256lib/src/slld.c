/* slld.c - Source Code for Cypress SPI Flash's Low Level Driver */

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


#include "slld.h"
#include "slld_hal.h"

#ifdef TRACE
#include "trace.h"
#endif

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
/* static variable to store SPI flash software protect status */
static DEV_SOFTWARE_PROTECT_STATUS sys_software_protect_status = FLASH_SOFTWARE_UNPROTECTED;
#endif

BYTE modebit_char;
BYTECOUNT g_PAGE_SIZE=PAGE_SIZE;

#ifdef INCL_ReadSecurityCmd
/******************************************************************************
 * slld_READ_SECURITY_Cmd - Read Security Register
 * This function issues the Read Security Register command to SPI Flash.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_ReadSecurityCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                      // device address given by system
        BYTE     *read_buf,                      // data buffer
        BYTECOUNT len_in_bytes                   // number of bytes
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_READ(device_num,SPI_READ_SECURITY_CMD, sys_addr, read_buf, len_in_bytes);
    return(status);
}
#endif 

#ifdef INCL_ProgramSecurityCmd
SLLD_STATUS slld_ProgramSecurityCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                 // device address given by system
        BYTE     *program_buf,              // data buffer
        BYTECOUNT len_in_bytes              // number of bytes
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PROGRAM_SECURITY_CMD, sys_addr, program_buf, len_in_bytes);
    return(status);
}

SLLD_STATUS slld_ProgramSecurityOp
(
        BYTE      device_num,                     //device number
        ADDRESS          sys_addr,                           // device address given by system
        BYTE             *program_buf,                       // data buffer
        BYTECOUNT        len_in_bytes,                       // number of bytes
        DEVSTATUS        *dev_status_ptr                     // variable to store device status
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_ProgramSecurityCmd(device_num, sys_addr, program_buf, len_in_bytes);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        // just in case SEOp is operated on a protected area

    return(status);
}
#endif

#ifdef INCL_EraseSecurityCmd
SLLD_STATUS slld_EraseSecurityCmd
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr                  // device address given by system
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_ERASE_SECURITY_CMD, sys_addr, BUFFER_NOT_USED, 0);
    return(status);
}

SLLD_STATUS slld_EraseSecurityOp
(
        BYTE            device_num,                     //device number
        ADDRESS     sys_addr,                 // device address given by system
        DEVSTATUS  *dev_status_ptr            // variable to store device status
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_EraseSecurityCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        // just in case SEOp is operated on a protected area

    return(status);
}
#endif

#ifdef INCL_ReadIdDualCmd
/******************************************************************************
 * slld_ReadIdDualCmd - Read ID from SPI Flash
 * This function issues the Read ID Dual command to SPI Flash and reads out the ID.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_ReadIdDualCmd
(
        BYTE                device_num,                     //device number
        ADDRESS                sys_addr,                             // device address given by system
        BYTE                *read_buf,                               // variable in which to store read data
        BYTE                modebit,                                 // mode bit
        BYTECOUNT        len_in_bytes                                // length in bytes
)
{
    SLLD_STATUS status = SLLD_OK;
    ADDRESS        tmp_addr;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    if (sys_addr < 0x2)
        tmp_addr = sys_addr;
    else
        tmp_addr = (ADDRESS)0;
    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_READID_DUAL_CMD, tmp_addr, read_buf, len_in_bytes);
    return(status);
}
#endif

#ifdef INCL_ReadIdQuadCmd
/******************************************************************************
 * slld_ReadIdQuadCmd - Read ID from SPI Flash
 * This function issues the Read ID Quad command to SPI Flash and reads out the ID.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_ReadIdQuadCmd
(
        BYTE                device_num,                     //device number
        ADDRESS                sys_addr,                             // device address given by system
        BYTE                *read_buf,                               // variable in which to store read data
        BYTE                modebit,                                 // mode bit
        BYTECOUNT        len_in_bytes                                // length in bytes
)
{
    SLLD_STATUS status = SLLD_OK;
    ADDRESS        tmp_addr;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    if (sys_addr < 0x2)
        tmp_addr = sys_addr;
    else
        tmp_addr = (ADDRESS)0;
    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_READID_QUAD_CMD, tmp_addr, read_buf, len_in_bytes);
    return(status);
}
#endif

#ifdef INCL_BE32KBCmd
/******************************************************************************
 * slld_BE32KB_Cmd - Block Erase 32KB
 * This function issues the Block Erase 32KB command to SPI Flash.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_BE32KBCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr                     // device address given by system
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_BE32KB_CMD, sys_addr, BUFFER_NOT_USED, 0);
    return(status);
}

SLLD_STATUS slld_BE32KBOp
(
        BYTE      device_num,                     //device number
        ADDRESS     sys_addr,                 // device address given by system
        DEVSTATUS  *dev_status_ptr            // variable to store device status
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_BE32KBCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);         // just in case SEOp is operated on a protected area

    return(status);
}
#endif

#ifdef INCL_WordReadQuadCmd
/******************************************************************************
 * slld_WordReadQuadCmd - Word Read Quad
 * This function issues the WordReadQuad command to SPI Flash and reads the word
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_WordReadQuadCmd
(
        BYTE              device_num,                     //device number
        ADDRESS                sys_addr,                             // device address given by system
        BYTE                *read_buf,                               // variable in which to store read data
        BYTE                modebit,                                 // mode bit
        BYTECOUNT        len_in_bytes                                // length in bytes
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_WORDREADQUAD_CMD, sys_addr, read_buf, len_in_bytes);

    return(status);
}
#endif

#ifdef INCL_ReadUniqueIDCmd
/******************************************************************************
 * slld_ReadUniqueIDCmd - Read Unique ID Number
 * This function issues the Read Unique ID command to SPI Flash and read 64-bit Unique Serial Number
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_ReadUniqueIDCmd
(
        BYTE      device_num,                     //device number
        BYTE      *read_buf        // data buffer
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    // read 64-bit number that is unique
    status = FLASH_RD(device_num,SPI_READ_UNIQUE_ID_CMD, ADDRESS_NOT_USED, read_buf, 8);

    return(status);
}
#endif

#ifdef INCL_OctalWordReadQuadCmd
/******************************************************************************
 * slld_OctalWordReadQuadCmd - Octal Word Read Quad
 * This function issues the Octal Word Read Quad command to SPI Flash
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_OctalWordReadQuadCmd
(
        BYTE      device_num,                     //device number
        ADDRESS                sys_addr,           // device address given by system
        BYTE                *read_buf,             // variable in which to store read data
        BYTE                modebit,               // mode bit
        BYTECOUNT        len_in_bytes              // length in bytes
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_OCTALWORDREADQUAD_CMD, sys_addr, read_buf, len_in_bytes);

    return(status);
}
#endif

#ifdef INCL_WriteVolatileCmd
/******************************************************************************
 * slld_WriteVolatileCmd - Write Enable for Volatile Status Register
 * This function issues the Write Volalite Status command to SPI Flash
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_WriteVolatileCmd
(
    BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WRITE_VOLATILE_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}

#endif 

#ifdef INCL_Read_IDCmd
/******************************************************************************
 *
 * slld_Read_IDCmd - Read ID from SPI Flash
 *
 * This function issues the Read_ID command to SPI Flash and reads out the ID.
 * This command sets the target device in software-unprotected state
 * so this function also sets sys_software_protect_status to FLASH_SOFTWARE_UNPROTECTED.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_Read_IDCmd
(
        BYTE      device_num,                     //device number
        BYTE    *target                      /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = FLASH_RD(device_num,SPI_READ_ID_CMD, (ADDRESS)ADDRESS_NOT_USED, target, 1);

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    sys_software_protect_status = FLASH_SOFTWARE_UNPROTECTED;
#endif

    return(status);
}
#endif /* INCL_Read_IDCmd */

#ifdef INCL_RDIDCmd
/******************************************************************************
 *
 * slld_RDIDCmd - Read Edentification from SPI Flash
 *
 * This function issues the RDID command to SPI Flash and reads out the ID.
 * On some devices this command is documented in the data sheet as
 * Release from deep power down and read electronic signature
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDIDCmd
(
        BYTE      device_num,                     //device number
        BYTE      *target,              /* variable in which to store read data */
        BYTECOUNT  len_in_bytes         /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RDID_CMD, (ADDRESS)ADDRESS_NOT_USED, target, len_in_bytes);

    return(status);
}
#endif /* INCL_RDIDCmd */

#ifdef INCL_READ_IDENTIFICATIONCmd
/******************************************************************************
 *
 * slld_Read_IdentificationCmd - Read ID from SPI Flash
 *
 * This function issues the Read_ID command to SPI Flash and reads out the ID.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_Read_IdentificationCmd
(
        BYTE      device_num,                     //device number
        BYTE    *target,                     /* variable in which to store read data */
        ADDRESS  addr                        /* address offset for the command */
)
{
    SLLD_STATUS status = SLLD_OK;
    ADDRESS    tmp_addr;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    if (addr < 0x2)
        tmp_addr = addr;
    else
        tmp_addr = (ADDRESS)0;

    status = FLASH_RD(device_num,SPI_READ_IDENTIFICATION_CMD, tmp_addr, target, 2);

    return(status);
}
#endif /* INCL_READ_IDENTIFICATIONCmd */

#ifdef INCL_RDSRCmd
/******************************************************************************
 *
 * slld_RDSRCmd - Read from Status Register
 *
 * This function issues the RDSR command to SPI Flash and reads from status register.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDSRCmd
(
        BYTE      device_num,                     //device number
        BYTE    *target                        /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_RD(device_num,SPI_RDSR_CMD, (ADDRESS)ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_RDSRCmd */

#ifdef INCL_RASPCmd
/******************************************************************************
 *
 * slld_RASPCmd - Read ASP Register
 *
 * This function issues the RASP command to SPI Flash and reads from ASP register.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RASPCmd
(
        BYTE      device_num,                     //device number
        WORD    *target                        /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RASP_CMD, (ADDRESS)ADDRESS_NOT_USED, (BYTE *)target, 2);

    return(status);
}
#endif /* INCL_RASPCmd */

#ifdef INCL_BRRDCmd
/******************************************************************************
 *
 * slld_BRRDCmd - Read Bank addressing register
 *
 * This function issues the BRRD command to SPI Flash and reads from the bank addressing register.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BRRDCmd
(
        BYTE      device_num,                     //device number
        BYTE    *target                        /* variable in which to store the bank addressing register value */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_BRRD_CMD, (ADDRESS)ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_BRRDCmd */

#ifdef INCL_ABRDCmd
/******************************************************************************
 *
 * slld_ABRDCmd - Read Autoboot register
 *
 * This function issues the read autoboot register command to SPI Flash and reads it.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ABRDCmd
(
        BYTE      device_num,                     //device number
        DWORD    *target           /* variable in which to store the autoboot register value */

)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_ABRD_CMD, (ADDRESS)ADDRESS_NOT_USED, (BYTE *)target, 4);

    return(status);
}
#endif /* INCL_ABRDCmd */

#ifdef INCL_ECCRDCmd
/******************************************************************************
 *
 * slld_ECCRDCmd - Read ECC Register per cache line
 *
 * This function issues the ECCRD command to SPI Flash and reads from ECC register per cache line
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ECCRDCmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr,                /* cache line address given by system */
        BYTE    *target                   /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_ECCRD_CMD, sys_addr, target, 1);

    return(status);
}
#endif /* INCL_ECCRDCmd */

#ifdef INCL_RPWDCmd
/******************************************************************************
 *
 * slld_RPWDCmd - Read password
 *
 * This function issues the RPWD command to SPI Flash and reads the password
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RPWDCmd
(
        BYTE      device_num,                     //device number
        BYTE    *target                 /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RPWD_CMD, (ADDRESS)ADDRESS_NOT_USED, target, 8);

    return(status);
}
#endif /* INCL_RPWDCmd */

#ifdef INCL_WRENCmd
/******************************************************************************
 *
 * slld_WRENCmd - Issue the write enable command
 *
 * This function issues the WREN command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRENCmd
(
    BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WREN_CMD, (ADDRESS)ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_WRENCmd */

#ifdef INCL_WRDICmd
/******************************************************************************
 *
 * slld_WRDICmd - Issue the write disable command
 *
 * This function issues the WRDI command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRDICmd
(
    BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WRDI_CMD, (ADDRESS)ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_WRDICmd */

#ifdef INCL_WRSRCmd
/******************************************************************************
 *
 * slld_WRSRCmd - Write to Status Register
 *
 * This function issues a write Status Register command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRSRCmd
(
        BYTE      device_num,                     //device number
        BYTE   *data_buf             /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WRSR_CMD, (ADDRESS)ADDRESS_NOT_USED, data_buf, 1);

    return(status);
}

/******************************************************************************
 *
 * slld_WRSROp - Write Status Register Operation
 *
 * Function writes data to Status Register.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRSROp
(
        BYTE  device_num,            /* device number */
        BYTE       *data_buf,                 /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRSRCmd(device_num, data_buf);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);          /* just in case WRSROp is operated on protected area */

    return(status);
}
#endif /* INCL_WRSRCmd */

void slld_VersionCmd(void)
{       
    printf("\n%s\n", SLLD_VERSION);
}

#ifdef INCL_SRSTCmd
/******************************************************************************
 *
 * slld_SRSTCmd - Software reset
 *
 * This function issues a software reset command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SRSTCmd
(
    BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;
    static unsigned int print_version= 0;

    if(!print_version) {
        slld_VersionCmd();
        print_version=1;
    }


#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SOFTWARE_RESET, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_SRSTCmd */

#ifdef INCL_PPB_PGCmd
/******************************************************************************
 *
 * slld_PPB_PGCmd - PPB Program
 *
 * This function issues the PPB Program command to SPI Flash and programs it.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPB_PGCmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr                  /* device address given by system */
)
{
    SLLD_STATUS  status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PPB_PG_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_PPB_PGOp - PPB Program Operation
 *
 * This function issues the PPB Program command to SPI Flash and programs it, then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPB_PGOp
(
        BYTE      device_num,                     //device number
        ADDRESS     sys_addr,                       /* device address given by system */
        DEVSTATUS  *dev_status_ptr                  /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_PPB_PGCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case PPOp is operated on protected area */

    return(status);
}
#endif /* INCL_PPB_PGCmd */

#ifdef INCL_DYB_PGCmd
/******************************************************************************
 *
 * slld_DYB_PGCmd - DYB Program
 *
 * This function issues the DYB Program command to SPI Flash and programs it.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DYB_PGCmd
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_DYB_PG_CMD, sys_addr, data_buf, 1);

    return(status);
}

/******************************************************************************
 *
 * slld_DYB_PGOp - DYB Program operation
 *
 * This function issues the DYB Program command to SPI Flash and programs it, then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DYB_PGOp
(
        BYTE      device_num,                     //device number
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE       *data_buf,                  /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr             /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_DYB_PGCmd(device_num, sys_addr, data_buf);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case PPOp is operated on protected area */

    return(status);
}
#endif /* INCL_DYB_PGCmd */

#ifdef INCL_ERS_SSPCmd
/******************************************************************************
 *
 * slld_ERS_SSPCmd - Sector Erase suspend ,Sector Erase/Program suspend for FL1K
 *
 * This function issues the Sector Erase suspend command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ERS_SSPCmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_ERS_SSP_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_ERS_SSPCmd */

#ifdef INCL_ERS_RESCmd
/******************************************************************************
 *
 * slld_ERS_RESCmd - Sector Erase resume ,Sector Erase/Program resume for FL1K
 *
 * This function issues the Sector Erase resume command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ERS_RESCmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_ERS_RES_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_ERS_RESCmd */

#ifdef INCL_RCSPCmd
/******************************************************************************
 *
 * slld_RCSPCmd - Recovery suspend
 *
 * This function issues the recovery suspend command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RCSPCmd
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_RCSP_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_RCSPCmd */

#ifdef INCL_RCRSCmd
/******************************************************************************
 *
 * slld_RCRSCmd - Recovery resume
 *
 * This function issues the recovery resume command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RCRSCmd
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_RCRS_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_RCRSCmd */

#ifdef INCL_RCVRCmd
/******************************************************************************
 *
 * slld_RCVRCmd - Initiate recovery mode (manually refreshing ECC)
 *
 * This function issues the initiate recovery mode command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RCVRCmd 
(        
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_RCVR_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_RCVRCmd */

#ifdef INCL_DPCmd
/******************************************************************************
 *
 * slld_DPCmd - Deep Power-down
 *
 * This function issues the Deep Power-down command to SPI Flash.
 * This command sets the target device in deep power-down state to reduce power consumption
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DPCmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_DP_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    sys_software_protect_status = FLASH_SOFTWARE_PROTECTED;
#endif

    return(status);
}
#endif /* INCL_DPCmd */

#ifdef INCL_BRACCCmd
/******************************************************************************
 *
 * slld_BRACCmd - Bank Register Acess
 *
 * This function issues the Bank Register Acess command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BRACCmd 
(        
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_BRAC_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_BRACCCmd */

#ifdef INCL_SPCmd
/******************************************************************************
 *
 * slld_SPCmd - Software Protect
 *
 * This function issues the Software Protect command to SPI Flash.
 * This command sets the target device in software-protected state
 * so this function also sets sys_software_protect_status[device_num] to FLASH_SOFTWARE_PROTECTED.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SPCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SP_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    sys_software_protect_status = FLASH_SOFTWARE_PROTECTED;
#endif

    return(status);
}
#endif /* INCL_SPCmd */

#ifdef INCL_RESCmd
/******************************************************************************
 *
 * slld_RESCmd - Release from Software Protect (Deep power-down)
 *
 * This function issues the Release from Software Protect command to SPI Flash.
 * This command sets the target device in software-unprotected state
 * so this function also sets sys_software_protect_status[device_num] to FLASH_SOFTWARE_UNPROTECTED.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RESCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

    status = FLASH_WR(device_num,SPI_RES_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    sys_software_protect_status = FLASH_SOFTWARE_UNPROTECTED;
#endif

    return(status);
}
#endif /* INCL_RESCmd */

#ifdef INCL_CLSRCmd
/******************************************************************************
 *
 * slld_ClearStatusRegisterCmd - Performs a clear status register command.
 *
 * This function issues the clear status register command to the SPI flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ClearStatusRegisterCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_CLSR_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_CLSRCmd */

#ifdef INCL_WRRCmd
/******************************************************************************
 *
 * slld_WRRCmd - Write to Status and Configure Registers
 *
 * This function issues the Write Registers command to the SPI Flash.
 * If configure value is NULL, the configure value will not be sent.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRRCmd
(
        BYTE  device_num,            /* device number */
        BYTE  *status_val,         /* variable containing data to program the status register */
        BYTE  *config_val,         /* variable containing data to program the configure register */
        BYTE  *status2_val         /* variable containing data to program the status register2 */
)
{
    SLLD_STATUS status = SLLD_OK;
    BYTE Buffer[3];

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    if(!status_val)return SLLD_ERROR;
    Buffer[0] = *status_val;
    if(status2_val != NULL)
    {
        if(!config_val || !status_val) return SLLD_ERROR;
        Buffer[1] = *config_val;
        Buffer[2] = *status2_val;
        status = FLASH_WR(device_num,SPI_WRR_CMD, ADDRESS_NOT_USED, Buffer, 3);
        return status;
    }
    if (config_val == NULL)
    {
        status = FLASH_WR(device_num,SPI_WRR_CMD, ADDRESS_NOT_USED, Buffer, 1);
    } else {
        Buffer[1] = *config_val;
        status = FLASH_WR(device_num,SPI_WRR_CMD, ADDRESS_NOT_USED, Buffer, 2);
    }

    return(status);
}

/******************************************************************************
 *
 * slld_WRROp - Write to Status and Config Registers
 *
 * This function issues the Write Registers command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRROp
(
        BYTE  device_num,            /* device number */
        BYTE       *status_val,          /* variable containing data to program the status register */
        BYTE       *config_val,          /* variable containing data to program the config register */
        BYTE       *status2_val,         /* variable containing data to program the status register2 */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRRCmd(device_num, status_val, config_val,status2_val);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case WRROp is operated on protected area */

    return(status);
}
#endif /* INCL_WRRCmd */


#ifdef INCL_WASPCmd
/******************************************************************************
 *
 * slld_WASPCmd - Write to ASP Register
 *
 * This function issues the Write ASP register command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WASPCmd
(
        BYTE  device_num,            /* device number */
        WORD    *asp_val                 /* variable containing data to program to the ASP register */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WASP_CMD, ADDRESS_NOT_USED, (BYTE *)asp_val, 2);

    return(status);
}

/******************************************************************************
 *
 * slld_WASPOP - Write to ASP Register operation
 *
 * This function issues the Write ASP register command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WASPOp
(
        BYTE  device_num,            /* device number */
        WORD       *asp_val,             /* variable containing data to program to the ASP register */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_WASPCmd(device_num, asp_val);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case WASPOp is operated on protected area */

    return(status);
}
#endif /* INCL_WASPCmd */

#ifdef INCL_BRWRCmd
/******************************************************************************
 *
 * slld_BRWRCmd - Write to the bank addressing register
 *
 * This function issues the Write bank addressing register command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BRWRCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *bnk_val                 /* variable containing data to program to the bank addressing register */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_BRWR_CMD, ADDRESS_NOT_USED, bnk_val, 1);

    return(status);
}
#endif /* INCL_BRWRCmd */

#ifdef INCL_ABWRCmd
/******************************************************************************
 *
 * slld_ABWRCmd - Write to the autoboot register
 *
 * This function issues the Write autoboot register command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ABWRCmd
(
        BYTE  device_num,            /* device number */
        DWORD    *abt_val                 /* variable containing data to program to the autoboot register */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_ABWR_CMD, ADDRESS_NOT_USED, (BYTE*)abt_val, 4);

    return(status);
}

/******************************************************************************
 *
 * slld_ABWROp - Write to the autoboot register
 *
 * This function issues the Write autoboot register command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ABWROp
(
        BYTE  device_num,            /* device number */
        DWORD       *abt_val,             /* variable containing data to program to the autoboot register */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_ABWRCmd(device_num, abt_val);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);          /* just in case ABWROp is operated on protected area */

    return(status);
}
#endif /* INCL_ABWRCmd */

#ifdef INCL_WPWDCmd
/******************************************************************************
 *
 * slld_WPWDCmd - Write the password
 *
 * This function issues the Write password command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WPWDCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *target                  /* variable containing data to program to the ASP password */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WPWD_CMD, ADDRESS_NOT_USED, target, 1);

    return(status);
}

/******************************************************************************
 *
 * slld_WPWDOp - Write to ASP password operation
 *
 * This function issues the Write password command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WPWDOp
(
        BYTE  device_num,            /* device number */
        BYTE       *target,              /* variable containing data to program to the ASP password */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_WPWDCmd(device_num, target);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);          /* just in case WPWDOp is operated on protected area */

    return(status);
}
#endif /* INCL_WPWDCmd */

#ifdef INCL_RCRCmd
/******************************************************************************
 *
 * slld_RCRCmd - Read from the configuration register
 *
 * This function issues the RCR command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RCRCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                   /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RCR_CMD, ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_RCRCmd */

#ifdef INCL_OTPRCmd
/******************************************************************************
 *
 * slld_OTPRCmd - Read from the OTP area
 *
 * This function issues the OTPR command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_OTPRCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_OTPR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_OTPRCmd */

#ifdef INCL_OTPPCmd
/******************************************************************************
 *
 * slld_OTPPCmd - OTP Program
 *
 * This function issues the OTP Program command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_OTPPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,                       /* device address given by system */
        BYTE    *data_buf                        /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_OTPP_CMD, sys_addr, data_buf, 1);

    return(status);
}

/******************************************************************************
 *
 * slld_OTPPOp - Performs a OTP Programming Operation.
 *
 * Function programs location in a page to the specified data.
 * Function issues all required commands and polls for completion.
 * Data size to program must be within PAZE_SIZE.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_OTPPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_OTPPCmd(device_num, sys_addr, data_buf);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);  /* just in case OTPPOp is operated on a protected area */

    return(status);
}
#endif /* INCL_OTPPCmd */

#ifdef INCL_ReadCmd
/******************************************************************************
 *
 * slld_ReadCmd - Read from SPI Flash
 *
 * This function issues the Read command to SPI Flash and reads from the array.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ReadCmd
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,          /* device address given by system */
        BYTE     *target,            /* variable in which to store read data */
        BYTECOUNT len_in_bytes       /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_READ_CMD, sys_addr, target, len_in_bytes);

    return(status);
}

#endif /* INCL_ReadCmd */

#ifdef INCL_Read_4BCmd
/******************************************************************************
 *
 * slld_Read_4BCmd - Read from SPI Flash in 4 bytes addressing scheme
 *
 * This function issues the Read_4B command to SPI Flash and reads from the array.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_Read_4BCmd
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,           /* device address given by system */
        BYTE     *target,             /* variable in which to store read data */
        BYTECOUNT len_in_bytes        /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_READ_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_Read_4BCmd */

#ifdef INCL_Fast_ReadCmd
/******************************************************************************
 *
 * slld_Fast_ReadCmd - Fast Read from SPI Flash
 *
 * This function issues the Fast Read command to SPI Flash and reads from the array.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_Fast_ReadCmd
(
        BYTE      device_num,                     //device number
        ADDRESS    sys_addr,              /* device address given by system */
        BYTE      *target,                /* variable in which to store read data */
        BYTECOUNT  len_in_bytes           /* number of bytes on which to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_FAST_READ_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_Fast_ReadCmd */

#ifdef INCL_Fast_Read_4BCmd
/******************************************************************************
 *
 * slld_Fast_Read_4BCmd - Fast Read from SPI Flash in 4 bytes addressing scheme
 *
 * This function issues the 4-bytes Fast Read command to SPI Flash and reads from the array.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_Fast_Read_4BCmd
(
        BYTE      device_num,                     //device number
        ADDRESS    sys_addr,              /* device address given by system */
        BYTE      *target,                /* variable in which to store read data */
        BYTECOUNT  len_in_bytes           /* number of bytes on which to operate */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_FAST_READ_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_Fast_Read_4BCmd */

#ifdef INCL_DUALOUTPUT_READCmd
/******************************************************************************
 *
 * slld_DualIOReadCmd - Read flash using dual IO
 *
 * This function issues the dual IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DualIOReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_DUALIO_RD_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DUALOUTPUT_READCmd */

#ifdef INCL_DUALOUTPUT_READ_4BCmd
/******************************************************************************
 *
 * slld_DualIORead_4BCmd - Read flash using dual IO in 4-bytes addressing scheme
 *
 * This function issues the dual IO read command and read the requested data using the 4-bytes
 * addressing scheme
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DualIORead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_DUALIO_RD_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DUALOUTPUT_READ_4BCmd */

#ifdef INCL_DORCmd
/******************************************************************************
 *
 * slld_DORCmd - Read flash using dual output
 *
 * This function issues the dual output read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_DOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DORCmd */

#ifdef INCL_4DORCmd
/******************************************************************************
 *
 * slld_4DORCmd - Read flash using dual output - 4 Bytes address
 *
 * This function issues the 4B dual output read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4DORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_4DOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_4DORCmd */

#ifdef INCL_QUADOUTPUT_READCmd
/******************************************************************************
 *
 * slld_QuadIOReadCmd - Read flash using quad IO
 *
 * This function issues the quad IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QuadIOReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_QUADIO_RD_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_QUADOUTPUT_READCmd */

#ifdef INCL_QUADOUTPUT_READ_4BCmd
/******************************************************************************
 *
 * slld_QuadIORead_4BCmd - Read flash using quad IO in 4-bytes addressing scheme
 *
 * This function issues the quad IO read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QuadIORead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_QUADIO_RD_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_QUADOUTPUT_READ_4BCmd */

#ifdef INCL_QORCmd
/******************************************************************************
 *
 * slld_QORCmd - Read flash using quad output
 *
 * This function issues the quad output read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_QOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_QORCmd */

#ifdef INCL_4QORCmd
/******************************************************************************
 *
 * slld_4QORCmd - Read flash using quad output in 4-bytes addressing scheme
 *
 * This function issues the quad output read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4QORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_4QOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_4QORCmd */

#ifdef INCL_DUALOUTPUT_HP_READCmd
/******************************************************************************
 *
 * slld_DualIOHPReadCmd - Read flash using dual IO HP
 *
 * This function issues the dual IO HP read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DualIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DUALIO_HPRD_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DUALOUTPUT_HP_READCmd */

#ifdef INCL_DUALOUTPUT_HP_READ_4BCmd
/******************************************************************************
 *
 * slld_DualIOHPRead_4BCmd - Read flash using dual IO HP in 4-bytes addressing scheme
 *
 * This function issues the dual IO HP read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DualIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DUALIO_HPRD_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DUALOUTPUT_HP_READ_4BCmd */

#ifdef INCL_DIORCmd
/******************************************************************************
 *
 * slld_DIORCmd - Read flash using dual IO
 *
 * This function issues the dual IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DIORCmd */

#ifdef INCL_4DIORCmd
/******************************************************************************
 *
 * slld_4DIORCmd - Read flash using dual IO - 4 Bytes address
 *
 * This function issues the 4B dual IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4DIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_4DIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_4DIORCmd */

#ifdef INCL_QUADOUTPUT_HP_READCmd
/******************************************************************************
 *
 * slld_QuadIOHPReadCmd - Read flash using quad IO HP
 *
 * This function issues quad IO HP read command and read the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QuadIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,     // device address given by system
        BYTE                *target,         // variable in which to store read data
        BYTE                modebit,         // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes        // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_QUADIO_HPRD_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_QUADOUTPUT_HP_READCmd */

#ifdef INCL_QUADOUTPUT_HP_READ_4BCmd
/******************************************************************************
 *
 * slld_QuadIOHPRead_4BCmd - Read flash using quad IO in 4-bytes addressing scheme
 *
 * This function issues the quad IO HP read command and reads the requested data in
 * 4-bytes addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QuadIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_QUADIO_HPRD_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_QUADOUTPUT_HP_READ_4BCmd */

#ifdef INCL_QIORCmd
/******************************************************************************
 *
 * slld_QIORCmd - Read flash using quad IO
 *
 * This function issues the quad IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE      modebit,        /* mode bit */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_QIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_QIORCmd */

#ifdef INCL_4QIORCmd
/******************************************************************************
 *
 * slld_4QIORCmd - Read flash using quad IO in 4-bytes addressing scheme
 *
 * This function issues the quad IO read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4QIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE      modebit,        /* mode bit */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_4QIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_4QIORCmd */


#ifdef INCL_DDR_Fast_ReadCmd
/******************************************************************************
 *
 * slld_DDR_Fast_ReadCmd - Read flash using fast DDR read
 *
 * This function issues DDR fast read command and read the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDR_Fast_ReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,     // device address given by system
        BYTE                *target,         // variable in which to store read data
        BYTE                modebit,         // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes        // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_FAST_READ_DDR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDR_Fast_ReadCmd */

#ifdef INCL_DDR_Fast_4BReadCmd
/******************************************************************************
 *
 * slld_DDR_Fast_ReadCmd - Read flash using fast DDR read  in 4-bytes
 *
 * This function issues DDR fast read command and read the requested data  in 4-bytes
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDR_Fast_4BReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_FAST_READ_DDR_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDR_Fast_4BReadCmd */

#ifdef INCL_DDR_DUALOUTPUT_HP_READCmd
/******************************************************************************
 *
 * slld_DualIOHPReadCmd - Read flash using DDR dual IO HP
 *
 * This function issues the DDR dual IO HP read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDR_DualIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DDR_DUALIO_HPRD_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDR_DUALOUTPUT_HP_READCmd */

#ifdef INCL_DDR_DUALOUTPUT_HP_READ_4BCmd
/******************************************************************************
 *
 * slld_DualIOHPRead_4BCmd - Read flash using DDR dual IO HP in 4-bytes addressing scheme
 *
 * This function issues the DDR dual IO HP read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDR_DualIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DDR_DUALIO_HPRD_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDR_DUALOUTPUT_HP_READ_4BCmd */

#ifdef INCL_DDR_QUADOUTPUT_HP_READCmd
/******************************************************************************
 *
 * slld_DualIOHPReadCmd - Read flash using DDR Quad IO HP
 *
 * This function issues the DDR Quad IO HP read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDR_QuadIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DDR_QUADIO_HPRD_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDR_QUADOUTPUT_HP_READCmd */

#ifdef INCL_DDR_QUADOUTPUT_HP_READ_4BCmd
/******************************************************************************
 *
 * slld_DualIOHPRead_4BCmd - Read flash using DDR Quad IO HP in 4-bytes addressing scheme
 *
 * This function issues the DDR Quad IO HP read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDR_QuadIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DDR_QUADIO_HPRD_4B_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDR_QUADOUTPUT_HP_READ_4BCmd */

#ifdef INCL_DDRQIORCmd
/******************************************************************************
 *
 * slld_DDRQIORCmd - Read flash using DDR Quad IO
 *
 * This function issues the DDR Quad IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDRQIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DDRQIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDRQIORCmd */

#ifdef INCL_4DDRQIORCmd
/******************************************************************************
 *
 * slld_4DDRQIORCmd - Read flash using DDR Quad IO in 4-bytes addressing scheme
 *
 * This function issues the DDR Quad IO read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4DDRQIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_4DDRQIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_4DDRQIORCmd */

#ifdef INCL_BulkEraseCmd
/******************************************************************************
 *
 * slld_BECmd - Bulk Erase ,Chip erase for FL1K
 *
 * This function issues the Bulk Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BECmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_BULKERASE_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_BEOp - Performs a Bulk Erase Operation ,Chip erase for FL1K
 *
 * Function erase all data in a device.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BEOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_BECmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case BEOp is operated on protected area */

    return(status);
}

SLLD_STATUS slld_BE1Cmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_BULKERASE1_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}

SLLD_STATUS slld_BE1Op
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_BE1Cmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case BEOp is operated on protected area */

    return(status);
}
#endif /* INCL_BulkEraseCmd */

#ifdef INCL_SECmd
/******************************************************************************
 *
 * slld_SECmd - Sector Erase
 *
 * This function issues the Sector Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SECmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr                     /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SE_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_SEOp - Performs a Sector Erase Operation
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SEOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef USE_4B_ADDR_BAR
    BYTE upper_addr;
    status = slld_BRACCmd(device_num);            /* BRAC, Bank Register Access */
    if(status != SLLD_OK)
        return(status);

    upper_addr = (BYTE)(sys_addr >> 24);
    status = slld_WRRCmd(device_num, &upper_addr,NULL,NULL);
    if(status != SLLD_OK)
        return(status);
#endif
    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

#ifdef USE_4B_ADDR_CMDS
    status = slld_SE_4BCmd(device_num, sys_addr);
#else
    status = slld_SECmd(device_num, sys_addr);
#endif
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case SEOp is operated on a protected area */

    return(status);
}
#endif /* INCL_SECmd */

#ifdef INCL_SE_4BCmd
/******************************************************************************
 *
 * slld_SE_4BCmd - Sector Erase using 4-bytes addressing scheme
 *
 * This function issues the Sector Erase command to SPI Flash using 4-bytes addressing scheme
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SE_4BCmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SE_4B_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_SE_4BOp - Performs a Sector Erase Operation using 4-bytes addressing scheme
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion using 4-bytes addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SE_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_SE_4BCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case SE_4BOp is operated on a protected area */

    return(status);
}
#endif /* INCL_SE_4BCmd */

#ifdef INCL_HBECmd
/******************************************************************************
 *
 * slld_HBECmd - Half Block Erase
 *
 * This function issues the Half Block Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_HBECmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr                     /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_HBE_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_HBEOp - Performs a Half Block Erase Operation
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_HBEOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

#ifdef USE_4B_ADDR_CMDS
    status = slld_4HBECmd(device_num, sys_addr);
#else
    status = slld_HBECmd(device_num, sys_addr);
#endif
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case HBEOp is operated on a protected area */

    return(status);
}
#endif /* INCL_HBECmd */

#ifdef INCL_4HBECmd
/******************************************************************************
 *
 * slld_4HBECmd - Half Block Erase using 4-bytes addressing scheme
 *
 * This function issues the Half Block Erase command to SPI Flash using 4-bytes addressing scheme
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4HBECmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4HBE_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}
#endif  /* INCL_4HBECmd */

#ifdef INCL_4BECmd
/******************************************************************************
 *
 * slld_4BECmd - BLOCK Erase - 4 Bytes address
 *
 * This function issues the 4B BLOCK Erase command to SPI Flash. 
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4BECmd 
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4BE_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

#endif  /* INCL_4HBECmd */

#ifdef INCL_CECmd
/******************************************************************************
 *
 * slld_CECmd - Chip erase
 *
 * This function issues the Chip Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_CECmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_CE_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_CEOp - Performs a chip Erase Operation 
 *
 * Function erase all data in a device.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_CEOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_CECmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case CEOp is operated on protected area */

    return(status);
}
#endif /* INCL_CECmd */

#ifdef INCL_CE1Cmd
/******************************************************************************
 *
 * slld_CE1Cmd - Chip erase alternate instruction
 *
 * This function issues the Chip Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_CE1Cmd 
(
        BYTE      device_num                     //device number
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_CE1_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_CE1Op - Performs a chip Erase Operation alternative instruction
 *
 * Function erase all data in a device.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_CE1Op
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_CE1Cmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case CEOp is operated on protected area */

    return(status);
}
#endif /* INCL_CE1Cmd */

#ifdef INCL_BLOCKERASECmd
/******************************************************************************
 *
 * slld_BECmd - BLOCK Erase
 *
 * This function issues the BLOCK Erase command to SPI Flash. The Block Erase
 * command sets all bits in the addressed 64 KB block to 1 (all bytes are FFh).
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BlockEraseCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_BLOCK_ERASE_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_BEOp - Performs a Bulk Erase Operation.
 *
 * Function erase all data in a device.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BlockEraseOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,             /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_BlockEraseCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case BEOp is operated on protected area */

    return(status);
}
#endif  //INCL_BLOCKERASECmd

#ifdef INCL_PPCmd
/******************************************************************************
 *
 * slld_PPCmd - Page Program
 *
 * This function issues a Page Program command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPCmd
(
        BYTE      device_num,                     //device number
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE       *data_buf,                  /* variable containing data to program */
        BYTECOUNT   len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS  status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PP_CMD, sys_addr, data_buf, len_in_bytes);

    return(status);
}

/******************************************************************************
 *
 * slld_PPOp - Performs a Page Programming Operation.
 *
 * This function programs location in a page to the specified data.
 * Function issues all required commands and polls for completion.
 * Data size to program must be within PAZE_SIZE.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    if(len_in_bytes > PAGE_SIZE)
    {
        status = SLLD_ERROR;  /* Data Bytes are larger than Page_Size */
        return(status);
    }

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_PPCmd(device_num, sys_addr, data_buf, len_in_bytes);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num); /* just in case PPOp is operated on protected area */

    return(status);
}
#endif /* INCL_PPCmd */

#ifdef INCL_PP_4BCmd
/******************************************************************************
 *
 * slld_PP_4BCmd - Page Program
 *
 * This function issues the Page Program command to SPI Flash in 4-bytes addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PP_4BCmd
(
        BYTE      device_num,                     //device number
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS  status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PP_4B_CMD, sys_addr, data_buf, len_in_bytes);

    return(status);
}

/******************************************************************************
 *
 * slld_PP_4BOp - Performs a Page Programming Operation using 4-bytes addressing scheme.
 *
 * Function programs location in a page to the specified data.
 * Function issues all required commands and polls for completion.
 * Data size to program must be within PAZE_SIZE.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PP_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    if(len_in_bytes > PAGE_SIZE)
    {
        status = SLLD_ERROR;  /* Data Bytes are larger than Page_Size */
        return(status);
    }

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_PP_4BCmd(device_num, sys_addr, data_buf, len_in_bytes);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case PPOp is operated on protected area */

    return(status);
}
#endif /* INCL_PP_4BCmd */

#ifdef INCL_P4ECmd
/******************************************************************************
 *
 * slld_P4ECmd - Erase 4K parameter Sector
 *
 * This function issues the P4E command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P4ECmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                  /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_P4E_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_P4EOp - Performs a 4K Sector Erase Operation.
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P4EOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_P4ECmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case P8EOp is operated on protected area */

    return(status);
}
#endif /* INCL_P4ECmd */

#ifdef INCL_P4E_4BCmd
/******************************************************************************
 *
 * slld_P4E_4BCmd - Erase 4K parameter Sector (4Byte Addr)
 *
 * This function issues the P4E4 command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P4E_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                          /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_P4E4_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_P4E_4BOp - Performs a 4K Sector Erase Operation (4Byte Address).
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P4E_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_P4E_4BCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case P8EOp is operated on protected area */

    return(status);
}
#endif /* INCL_P4E_4BOp */

#ifdef INCL_P8ECmd
/******************************************************************************
 *
 * slld_P8ECmd - Erase 8K parameter Sector
 *
 * This function issues the P8E command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P8ECmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                  /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_P8E_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_P8EOp - Performs a 8K Sector Erase Operation.
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P8EOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_P8ECmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case P8EOp is operated on protected area */

    return(status);
}
#endif /* INCL_P8ECmd */

#ifdef INCL_P8E_4BCmd
/******************************************************************************
 *
 * slld_P8ECmd - Erase 8K parameter Sector using 4-bytes addressing scheme
 *
 * This function issues the P8E command to the SPI Flash using 4-bytes addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P8E_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                   /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_P8E_4B_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_P8E_4BOp - Performs a 8K Sector Erase Operation using 4-bytes addressing scheme.
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_P8E_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_P8E_4BCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        /* just in case P8E_4BOp is operated on protected area */

    return(status);
}
#endif /* INCL_P8E_4BCmd */

#ifdef INCL_QPPCmd
/******************************************************************************
 *
 * slld_QPPCmd - Page Program using quad IO
 *
 * This function issues the Page Program command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_QPP_CMD, sys_addr, data_buf, len_in_bytes);

    return(status);
}

/******************************************************************************
 *
 * slld_QPPOp - Performs a Quad-Io Page Programming Operation.
 *
 * Function programs location in a page to the specified data.
 * Function issues all required commands and polls for completion.
 * Data size to program must be within PAZE_SIZE.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    if(len_in_bytes > PAGE_SIZE)
    {
        status = SLLD_ERROR;  /* Data Bytes are larger than Page_Size */
        return(status);
    }

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_QPPCmd(device_num, sys_addr, data_buf, len_in_bytes);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case QPPOp is operated on protected area */

    return(status);
}
#endif /* INCL_QPPCmd */

#ifdef INCL_QPP_4BCmd
/******************************************************************************
 *
 * slld_QPP_4BCmd - Page Program using quad IO in 4-bytes addressing scheme
 *
 * This function issues the Page Program command to the SPI Flash in 4-bytes addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPP_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE       *data_buf,                  /* variable containing data to program */
        BYTECOUNT   len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_QPP_4B_CMD, sys_addr, data_buf, len_in_bytes);

    return(status);
}

/******************************************************************************
 *
 * slld_QPP_4BOp - Performs a Quad-Io Page Programming Operation using 4-bytes addressing scheme.
 *
 * Function programs location in a page to the specified data.
 * Function issues all required commands and polls for completion.
 * Data size to program must be within PAZE_SIZE.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPP_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    if(len_in_bytes > PAGE_SIZE)
    {
        status = SLLD_ERROR;  /* Data Bytes are larger than Page_Size */
        return(status);
    }

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_QPP_4BCmd(device_num, sys_addr, data_buf, len_in_bytes);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case QPPOp is operated on protected area */

    return(status);
}
#endif /* INCL_QPP_4BCmd */

#ifdef INCL_QPP2md
/******************************************************************************
 *
 * slld_QPP2Cmd - Page Program using quad IO
 *
 * This function issues the QPP command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPP2Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_QPP2_CMD, sys_addr, data_buf, len_in_bytes);

    return(status);
}
#endif /* INCL_QPP2md */

#ifdef INCL_MPMCmd
/******************************************************************************
 *
 * slld_MPMCmd - Charges the charge pumps for fast Multi-IO operation
 *
 * This function charges the charge pumps for fast Multi-IO operation.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_MPMCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_MPM_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_MPMCmd */

/******************************************************************************
 *
 * slld_Poll - Polls flash device for embedded operation completion
 *
 * This function polls the Flash device to determine when an embedded
 * operation is finished.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_Poll
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr              /* variable to store device status */
)
{
    SLLD_STATUS       status = SLLD_OK;

    *dev_status_ptr = dev_status_unknown;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    do
    {
        status = slld_StatusGet(device_num, dev_status_ptr);
        if(status != SLLD_OK)
            return(status);
    }
    while(*dev_status_ptr == dev_busy);

    return(status);
}

/******************************************************************************
 *
 * slld_StatusGet - Determines Flash Status
 *
 * This function gets the device status from the SPI flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_StatusGet
(
        BYTE  device_num,            /* device number */
        DEVSTATUS*  dev_status_ptr                  /* variable to store device status */
)
{
    SLLD_STATUS  status = SLLD_OK;
    BYTE poll_data;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = slld_RDSRCmd(device_num, &poll_data);
    if(status != SLLD_OK)
        return(status);

    // Check if steady state
    if (poll_data & B0_MASK) {                     // If b0 = 1 then device is busy
#ifdef STATUS_IN_SR2
        status = slld_RDSR2Cmd(device_num, &poll_data);
        if(status != SLLD_OK)
            return(status);
#endif
        // If b5 = 1 then there was an erase error
        if ((STATUS_ERROR_FLAGS & HAS_STATBIT5_ERROR) && (poll_data & B5_MASK))
            *dev_status_ptr = dev_erase_error;
        // If b6 = 1 then there was a program error
        else if ((STATUS_ERROR_FLAGS & HAS_STATBIT6_ERROR) && (poll_data & B6_MASK))
            *dev_status_ptr = dev_program_error;
        else
            *dev_status_ptr = dev_busy;
    }
    else
    {
        // If b0 = 0 then device is not busy
        *dev_status_ptr = dev_not_busy;
    }

    return (status);
}

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
/******************************************************************************
 *
 * slld_SoftwareProtectStatusGet - Determines Flash Software Protection Status
 *
 * This function determines the flash software protection status.
 *
 * RETURNS: SLLD_OK.
 *
 */
SLLD_STATUS slld_SoftwareProtectStatusGet    ///xbill ????, how to add device_num?
(
        DEV_SOFTWARE_PROTECT_STATUS*  dev_softwareprotect_status_ptr  /* variable to store device software protect status */
)
{
    SLLD_STATUS status = SLLD_OK;

    *dev_softwareprotect_status_ptr = sys_software_protect_status;

    return(status);
}
#endif /* INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK */

#ifdef INCL_BufferedProgramOp
/******************************************************************************
 *
 * slld_BufferedProgramOp - Performs a Programming Operation.
 *
 * Function programs data to the specfied address.
 * Function issues all required commands and polls for completion. Address doesn't have to be
 * page aligned.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BufferedProgramOp
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                   /* device address given by system */
        BYTE      *data_buf,                   /* variable containing data to program */
        BYTECOUNT  len_in_bytes,               /* number of bytes on which to operate */
        DEVSTATUS *dev_status_ptr              /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;
    ADDRESS     current_sys_addr;          /* current address to which program operation operates */
    BYTECOUNT   current_len_in_bytes;      /* number of bytes to program at a time */
    BYTE       *current_data_ptr;          /* pointer to data to program */

    /* initial setting of variables */
    current_sys_addr = sys_addr;
    current_data_ptr = data_buf;

    current_len_in_bytes = PAGE_SIZE - (sys_addr & PAGE_MASK);
    if (current_len_in_bytes > len_in_bytes)
        current_len_in_bytes = len_in_bytes;

    do
    {
        status = slld_PPOp(device_num, current_sys_addr, current_data_ptr, current_len_in_bytes, dev_status_ptr);
        if(status != SLLD_OK)
            return(status);

        len_in_bytes -= current_len_in_bytes;

        /* set variables for next programming */
        current_sys_addr += current_len_in_bytes;
        current_data_ptr += current_len_in_bytes;

        if (len_in_bytes > PAGE_SIZE)
        {
            current_len_in_bytes = PAGE_SIZE;
        }
        else
        {
            current_len_in_bytes = len_in_bytes;
        }
    }
    while (len_in_bytes != 0);

    status = slld_WRDICmd(device_num);          /* just in case BufferedProgramOp is operated on protected area */

    return(status);
}
#endif /* INCL_BufferedProgramOp */

#ifdef INCL_BufferedProgram_4BOp
/******************************************************************************
 *
 * slld_BufferedProgram_4BOp - Performs a Programming Operation using 4-bytes addressing scheme.
 *
 * Function programs data to the specfied address.
 * Function issues all required commands and polls for completion. Address doesn't have to be
 * page aligned.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BufferedProgram_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE       *data_buf,                  /* variable containing data to program */
        BYTECOUNT   len_in_bytes,              /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr             /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;
    ADDRESS     current_sys_addr;          /* current address to which program operation operates */
    BYTECOUNT   current_len_in_bytes;      /* number of bytes to program at a time */
    BYTE       *current_data_ptr;          /* pointer to data to program */

    /* initial setting of variables */
    current_sys_addr = sys_addr;
    current_data_ptr = data_buf;

    current_len_in_bytes = PAGE_SIZE - (sys_addr & PAGE_MASK);
    if (current_len_in_bytes > len_in_bytes)
        current_len_in_bytes = len_in_bytes;

    do
    {
        status = slld_PP_4BOp(device_num, current_sys_addr, current_data_ptr, current_len_in_bytes, dev_status_ptr);
        if(status != SLLD_OK)
            return(status);

        len_in_bytes -= current_len_in_bytes;

        /* set variables for next programming */
        current_sys_addr += current_len_in_bytes;
        current_data_ptr += current_len_in_bytes;

        if (len_in_bytes > PAGE_SIZE)
        {
            current_len_in_bytes = PAGE_SIZE;
        }
        else
        {
            current_len_in_bytes = len_in_bytes;
        }
    }
    while (len_in_bytes != 0);

    status = slld_WRDICmd(device_num);          /* just in case BufferedProgram_4BOp is operated on protected area */

    return(status);
}
#endif /* INCL_BufferedProgram_4BOp */

#ifdef INCL_BlockProtectOp
/******************************************************************************
 *
 * slld_BlockProtectOp - Performs a Block Protection Operation.
 *
 * Function sets Block Protect bits to protect specified memory area.
 * Function issues all required commands and polls for completion.
 *
 * Valid <bpb_value> values are:  (please see the datasheet for each device)
 *  0x00 - 0x03 : for S25FL002D, S25FL001D...
 *  0x00 - 0x07 : for S25FL004D, S25FL032P...
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_BlockProtectOp
(
        BYTE  device_num,            /* device number */
        BYTE        bpb_value,                 /* block protect bit value */
        DEVSTATUS  *dev_status_ptr             /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;
    BYTE        current_status_register;          /* current state of Status Register */
    BYTE        new_status_register;              /*new state of Status Register */

    status = slld_RDSRCmd(device_num, &current_status_register);
    if(status != SLLD_OK)
        return(status);

    new_status_register = ((current_status_register & (~BLOCK_PROTECT_BITS_MASK)) | (bpb_value << 2) );

#ifdef BLOCK_PROTECT_USE_WRAR
// add by pzhu, write volitale register 
    status = slld_WRAR_Op(device_num, SR1V, &new_status_register, dev_status_ptr);
#else
    status = slld_WRSROp(device_num, &new_status_register, dev_status_ptr);
#endif

    return(status);
}
#endif /* INCL_BlockProtectOp */

#ifdef INCL_QuadWriteOp
SLLD_STATUS slld_QuadWriteOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;
    BYTECOUNT length;

    while(len_in_bytes)
    {
        if(len_in_bytes < PAGE_SIZE)
            length = len_in_bytes;
        else
            length = PAGE_SIZE;

        status = slld_QPPOp(device_num, sys_addr, data_buf, length, dev_status_ptr);
        if(status != SLLD_OK)
            return(status);

        len_in_bytes -= length;
        sys_addr += length;
        data_buf += length;
    }

    return(status);
}
#endif /* INCL_QuadWriteOp */

#ifdef INCL_RDSR2Cmd
/******************************************************************************
 *
 * slld_RDSR2Cmd - Read from Status Register-2
 *
 * This function issues the RDSR2 command to SPI Flash and reads from status register.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDSR2Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                        /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RDSR2_CMD, (ADDRESS)ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_RDSR2Cmd */

#ifdef INCL_DLPRDCmd
/******************************************************************************
 *
 * slld_DLPRDCmd - Read Data Learning Pattern
 *
 * This function issues the DLPRD command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DLPRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable in which to store data */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_DLPRD_CMD, ADDRESS_NOT_USED, data_buf, len_in_bytes);

    return(status);
}
#endif /* INCL_DLPRDCmd */

#ifdef INCL_PNVDLRCmd
/******************************************************************************
 *
 * slld_PNVDLRCmd - Program NV Data Learning Register
 *
 * This function issues the PNVDLR command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PNVDLRCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);
    status = FLASH_WR(device_num,SPI_PNVDLR_CMD, ADDRESS_NOT_USED, data_buf, len_in_bytes);

    return(status);
}
#endif /* INCL_PNVDLRCmd */

#ifdef INCL_WVDLRCmd
/******************************************************************************
 *
 * slld_WVDLRCmd - Write Volatile Data Learning Register
 *
 * This function issues the WVDLR command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WVDLRCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = FLASH_WR(device_num,SPI_WVDLR_CMD, ADDRESS_NOT_USED, data_buf, len_in_bytes);

    return(status);
}
#endif /* INCL_WVDLRCmd */

#ifdef INCL_PGSPCmd
/******************************************************************************
 *
 * slld_PGSPCmd - Program Suspend
 *
 * This function issues the PGSP command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PGSPCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PGSP_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_PGSPCmd */

#ifdef INCL_PGRSCmd
/******************************************************************************
 *
 * slld_PGRSCmd - Program Resume
 *
 * This function issues the PGRS command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PGRSCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PGRS_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_PGRSCmd */

#ifdef INCL_PLBWRCmd
/******************************************************************************
 *
 * slld_PLBWRCmd - PPB Lock Bit Write
 *
 * This function issues the PLBWR command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PLBWRCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PLBWR_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_PLBWRCmd */

#ifdef INCL_PLBRDCmd
/******************************************************************************
 *
 * slld_PLBRDCmd - PPB Lock Bit Read
 *
 * This function issues the PLBRD command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PLBRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_PLBRD_CMD, ADDRESS_NOT_USED, data_buf, len_in_bytes);

    return(status);
}
#endif /* INCL_PLBRDCmd */

#ifdef INCL_DYB_RDCmd
/******************************************************************************
 *
 * slld_DYB_RDCmd - DYB Read
 *
 * This function issues the DYB Read command to SPI Flash and read data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DYB_RDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* data buffer to write */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_DYB_RD_CMD, sys_addr, data_buf, 1);

    return(status);
}
#endif /* INCL_DYB_RDCmd */

#ifdef INCL_PPB_RDCmd
/******************************************************************************
 *
 * slld_PPB_RDCmd - PPB Read
 *
 * This function issues the PPB Read command to SPI Flash and read data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPB_RDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_PPB_RD_CMD, sys_addr, data_buf, 1);

    return(status);
}
#endif /* INCL_PPB_RDCmd */

#ifdef INCL_PPB_ERSCmd
/******************************************************************************
 *
 * slld_PPB_ERSCmd - PPB Erase
 *
 * This function issues the PPB Erase command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPB_ERSCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PPB_ERS_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_PPB_ERSCmd */

#ifdef INCL_PASSRDCmd
/******************************************************************************
 *
 * slld_PASSRDCmd - Password Read
 *
 * This function issues the Password Read command to SPI Flash and read data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PASSRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *data_buf                       /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_PASSRD_CMD, ADDRESS_NOT_USED, data_buf, 8);        // 64-bit password

    return(status);
}
#endif /* INCL_PASSRDCmd */

#ifdef INCL_PASSPCmd
/******************************************************************************
 *
 * slld_PASSPCmd - Password Program
 *
 * This function issues the Password Program command to SPI Flash and write data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PASSPCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *data_buf                       /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PASSP_CMD, ADDRESS_NOT_USED, data_buf, 8);        // 64-bit password

    return(status);
}
#endif /* INCL_PASSPCmd */

#ifdef INCL_PASSUCmd
/******************************************************************************
 *
 * slld_PASSUCmd - Password Unlock
 *
 * This function issues the Password Unlock command to SPI Flash and unlock data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PASSUCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *data_buf                       /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PASSU_CMD, ADDRESS_NOT_USED, data_buf, 8);        // 64-bit password

    return(status);
}
#endif /* INCL_PASSUCmd */

#ifdef INCL_RDARCmd
/******************************************************************************
 *
 * slld_RDARCmd - Read any device register-non-volatile and volatile.
 *
 * This function issues the read any register command and reads the requested data in 1-bytes
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDARCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS             reg_addr,         /* register address given by device*/
        BYTE                *target           /*variable in which to store read data*/
)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_RD(device_num,SPI_RDAR_CMD, reg_addr, target, 1);

    return(status);
}
#endif //INCL_RDARCmd

#ifdef INCL_WRARCmd
/******************************************************************************
 *
 * slld_WRARCmd - Write any device register-non-volatile and volatile.
 *
 * This function issues the write any register command and write 1-byte
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRARCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS             reg_addr,         /*register address given by device*/
        BYTE                *data_buf         /*variable containing data to program*/
)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_WR(device_num,SPI_WRAR_CMD, reg_addr, data_buf, 1);

    return(status);
}

/******************************************************************************
 *
 * slld_WRAR_Op - Write any device register-non-volatile and volatile operation
 *
 * This function issues the write any register command and write 1-byte
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRAR_Op
(
        BYTE  device_num,            /* device number */
        ADDRESS             reg_addr,         /*register address given by device*/
        BYTE                *data_buf,        /*variable containing data to program*/
        DEVSTATUS           *dev_status_ptr   /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRARCmd(device_num, reg_addr,data_buf);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case PPOp is operated on protected area */

    return(status);
}
#endif //INCL_WRARCmd

#ifdef INCL_RSTENCmd
/******************************************************************************
 *
 * slld_RSTENCmd -Reset Enable
 *
 * This function issues the Reset Enable command which required immediately before a
 * Reset command(RST).
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RSTENCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_WR(device_num,SPI_RSTEN_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif //INCL_RSTENCmd

#ifdef INCL_RSTCmd
/******************************************************************************
 *
 * slld_RSTCmd -Reset Reset
 *
 * This function issues the Software Reset command immediately following a RSTEN command,
 * initiates the software reset process.
 *
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RSTCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_WR(device_num,SPI_RESET_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif //INCL_RSTCmd

#ifdef INCL_RDQIDCmd
/******************************************************************************
 *
 * slld_RDQIDCmd - Read access manufacturer id ,device id and CFI in Quad All Mode
 *
 * This function issues the RDQID command.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDQIDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *target,              /* variable in which to store read data */
        BYTECOUNT  len_in_bytes         /* number of bytes to read */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RDQID_CMD, (ADDRESS)ADDRESS_NOT_USED, target, len_in_bytes);

    return(status);
}
#endif /* INCL_RDQIDCmd */


#ifdef INCL_EESCmd
/******************************************************************************
 *
 * slld_ESSCmd - Evaluate Erase Status command.
 *
 * This function issues Evaluate Erase status command,verify that the last erase
 * operation on the addressed sector was completed successfully,update SR2V[2] bit
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_ESSCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr      /* device address given by system*/
)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_WR(device_num,SPI_EES_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}
#endif //INCL_EESCmd

#ifdef INCL_DYBRD_Cmd
/******************************************************************************
 *
 * slld_DYBRD_Cmd - DYB Read
 *
 * This function issues the DYB Read command to SPI Flash and read data,the command is followed
 * by the 24 or 32-bit address,depending on the address length configuration CR2V[7],selecting location
 * zero within the desired sector.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DYBRD_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* data buffer to write */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_DYBRD_CMD, sys_addr, data_buf, 1);

    return(status);
}
#endif /* INCL_DYBRD_Cmd */

#ifdef INCL_PPBRD_Cmd
/******************************************************************************
 *
 * slld_PPBRD_Cmd - PPB Read
 *
 * This function issues the PPB Read command to SPI Flash and read data.the command is followed
 * by the 24 or 32-bit address,depending on the address length configuration CR2V[7],selecting location
 * zero within the desired sector.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPBRD_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_PPBRD_CMD, sys_addr, data_buf, 1);

    return(status);
}
#endif /* INCL_PPBRD_Cmd */

#ifdef INCL_PPBP_Cmd
/******************************************************************************
 *
 * slld_PPBP_Cmd - PPB Program
 *
 * This function issues the PPB Program command to SPI Flash and programs it.the command is followed
 * by the 24 or 32-bit address,depending on the address length configuration CR2V[7]
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPBP_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                  /* device address given by system */
)
{
    SLLD_STATUS  status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PPBP_CMD, sys_addr, BUFFER_NOT_USED, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_PPBP_Op - PPB Program Operation
 *
 * This function issues the PPB Program command to SPI Flash and programs it, then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PPBP_Op
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                       /* device address given by system */
        DEVSTATUS  *dev_status_ptr                  /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_PPBP_Cmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case PPOp is operated on protected area */

    return(status);
}
#endif /* INCL_PPBP_Cmd */

#ifdef INCL_DYBWR_Cmd
/******************************************************************************
 *
 * slld_DYBWR_Cmd - DYB write
 *
 * This function issues the DYB write command to SPI Flash and programs it.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DYBWR_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* variable containing data to program */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_DYBWR_CMD, sys_addr, data_buf, 1);

    return(status);
}

/******************************************************************************
 *
 * slld_DYBWR_Op - DYB Write operation
 *
 * This function issues the DYB write command to SPI Flash and programs it, then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DYBWR_Op
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE       *data_buf,                  /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr             /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_DYBWR_Cmd(device_num, sys_addr, data_buf);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);/* just in case PPOp is operated on protected area */

    return(status);
}
#endif /* INCL_DYBWR_Cmd */

#ifdef INCL_EPR_Cmd
/******************************************************************************
 *
 * slld_EPR_Cmd - Sector Erase resume  ,Sector Erase/Program resume for FSS,only available when set CR3NV[2] to 1.
 * if CR3NV[2] is 0 ,can using resume command 0x70 or 0x8a slld_ERS_RESCmd() or slld_PGRSCmd().
 * This function issues the Sector Erase resume command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_EPR_Cmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_EPR_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_EPR_Cmd */

#ifdef INCL_EPS_Cmd
/******************************************************************************
 *
 * slld_EPS_Cmd - Sector Erase suspend ,Sector Erase/Program suspend suspend for FSS
 *
 * This function issues the Sector Erase suspend command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_EPS_Cmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_EPS_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_EPS_Cmd */

#ifdef INCL_4BAM_Cmd
/******************************************************************************
 *
 * slld_4BAM_Cmd -4-byte Address Mode (4BAM) command sets the volatile Address Length bit (CR2V[7]) to 1 to
 * change most 3-byte address commands to require 4 bytes of address.
 *
 * This function issues the 4-byte Address Mode command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4BAM_Cmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4BAM_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_4BAM_Cmd */

SLLD_STATUS slld_ReadOp
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,          /* device address given by system */
        BYTE     *target,            /* variable in which to store read data */
        BYTECOUNT len_in_bytes       /* number of bytes to read */
)
{
    SLLD_STATUS         status = SLLD_OK;
    unsigned long        length;
#ifdef USE_4B_ADDR_BAR
    BYTE upper_addr;
#endif
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    while (len_in_bytes)
    {
#ifdef CONTROLLER_BUFFER_SIZE
        if (len_in_bytes < CONTROLLER_BUFFER_SIZE)
            length = len_in_bytes;
        else
            length = CONTROLLER_BUFFER_SIZE;
#else
        length = len_in_bytes;
#endif
#ifdef USE_4B_ADDR_BAR
#ifndef USE_4B_ADDR_CMDS
        status = slld_BRACCmd(device_num);            /* BRAC, Bank Register Access */
        if(status != SLLD_OK)
            return(status);

        upper_addr = (BYTE)(sys_addr >> 24);
        status = slld_WRRCmd(device_num, &upper_addr,NULL,NULL);
        if(status != SLLD_OK)
            return(status);
#endif
#endif

#ifdef USE_4B_ADDR_CMDS
#ifdef USE_FAST_READ
        status = FLASH_RD(device_num,SPI_FAST_READ_4B_CMD, sys_addr, target, length);
#elif defined USE_DUAL_READ
        status = FLASH_RD(device_num,SPI_DUALIO_RD_4B_CMD, sys_addr, target, length);
#elif defined USE_QUAD_READ
        status = FLASH_RD(device_num,SPI_QUADIO_RD_4B_CMD, sys_addr, target, length);
#else
        status = FLASH_RD(device_num,SPI_READ_4B_CMD, sys_addr, target, length);
#endif
#else //#ifdef USE_4B_ADDR_CMDS
#ifdef USE_FAST_READ
        status = FLASH_RD(device_num,SPI_FAST_READ_CMD, sys_addr, target, length);
#elif defined USE_DUAL_READ
        status = FLASH_RD(device_num,SPI_DUALIO_RD_CMD, sys_addr, target, length);
#elif defined USE_QUAD_READ
        status = FLASH_RD(device_num,SPI_QUADIO_RD_CMD, sys_addr, target, length);
#else
        status = FLASH_RD(device_num,SPI_READ_CMD, sys_addr, target, length);
#endif
#endif

        len_in_bytes -= length;
        sys_addr += length;
        target += length;
    }

    return(status);
}


SLLD_STATUS slld_WriteOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;
    unsigned long length;
#ifdef USE_4B_ADDR_BAR
    BYTE upper_addr;
#endif

    while(len_in_bytes)
    {
#ifdef USE_4B_ADDR_BAR
#ifndef USE_4B_ADDR_CMDS
        status = slld_BRACCmd(device_num);            /* BRAC, Bank Register Access */
        if(status != SLLD_OK)
            return(status);

        upper_addr = (BYTE)(sys_addr >> 24);
        status = slld_WRRCmd(device_num, &upper_addr,NULL,NULL);
        if(status != SLLD_OK)
            return(status);
#endif
#endif

        if(len_in_bytes < PAGE_SIZE)
            length = len_in_bytes;
        else
            length = PAGE_SIZE;

        status = slld_WRENCmd(device_num);
        if(status != SLLD_OK)
            return(status);
#ifdef USE_4B_ADDR_CMDS
#ifdef USE_QUAD_WRITE
        status = slld_QPP_4BCmd(device_num, sys_addr, data_buf, length);
#else
        status = slld_PP_4BCmd(device_num, sys_addr, data_buf, length);
#endif
#else //#ifdef USE_4B_ADDR_CMDS
#ifdef USE_QUAD_WRITE
        status = slld_QPPCmd(device_num, sys_addr, data_buf, length);
#else
        status = slld_PPCmd(device_num, sys_addr, data_buf, length);
#endif
#endif

        if(status != SLLD_OK)
            return(status);

        status = slld_Poll(device_num, dev_status_ptr);
        if(status != SLLD_OK)
            return(status);

        status = slld_WRDICmd(device_num);
        if(status != SLLD_OK)
            return(status);

        len_in_bytes -= length;
        sys_addr += length;
        data_buf += length;
    }

    return(status);
}

#ifdef INCL_SPI_READSFDPCMD
/******************************************************************************
 * slld_ReadSFDPCmd - Read Serial Flash Discoverable Parameter
 * This function issues the Read SFDP command to SPI Flash
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_ReadSFDPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        // device address given by system
        BYTE      *read_buf         // data buffer
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    // read 256-byte serial flash discoverable parameter (SFDP) register
    status = FLASH_RD(device_num,SPI_READ_SFDP_CMD, sys_addr, read_buf, 256);

    return(status);
}
#endif  // INCL_SPI_READSFDPCMD

#ifdef  INCL_SPI_READMODERESETCMD
/******************************************************************************
 * slld_ReadModeResetCmd - Continuous Read Mode Reset
 * This function issues the Read Mode Reset command to SPI Flash
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_ReadModeResetCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;
    BYTE Buffer[1];

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    Buffer[0] = 0xff;
    status = FLASH_WR(device_num,SPI_READMODE_RESET_CMD, ADDRESS_NOT_USED, Buffer, 1);

    return(status);
}
#endif

#ifdef INCL_SPI_BURSTWRAPCMD
/******************************************************************************
 * slld_SetBurstWrapCmd - Set Burst with Wrap
 * This function issues the Set Burst with Wrap command to SPI Flash
 * for FSS family device , this function write CR4V register to enable
 * and disable the wrapped read feature and set the wrap boundary.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_SetBurstWrapCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *wrapbit_buf                     // variable in which to store wrapbit data
)
{
    SLLD_STATUS status = SLLD_OK;
#ifndef BURSTWRAP_SIZE_1
    BYTE buf[4]={0xff,0xff,0xff,0xff};
    buf[3] = *wrapbit_buf;
#endif
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
#ifdef BURSTWRAP_SIZE_1
    status = FLASH_WR(device_num,SPI_SBL_CMD, (ADDRESS)ADDRESS_NOT_USED, wrapbit_buf, 1);
#else
    status = FLASH_WR(device_num,SPI_SETBURSTWRAP_CMD, ADDRESS_NOT_USED, buf, 4);
#endif

    return(status);
}
#endif

#ifdef INCL_RDSR3Cmd
/******************************************************************************
 *
 * slld_RDSR3Cmd - Read from Status Register-3
 *
 * This function issues the RDSR2 command to SPI Flash and reads from status register.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDSR3Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                        /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RDSR3_CMD, (ADDRESS)ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_RDSR3Cmd */

#ifdef INCL_WRSRegCmd
/******************************************************************************
 *
 * slld_WRRCmd - Write to Status Registers
 *
 * This function issues the Write Registers command to the SPI Flash.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR or SLLD_ERROR
 *
 */
SLLD_STATUS slld_WRSRegCmd
(
        BYTE  device_num,            /* device number */
        BYTE        *status_val,         /* variable array containing data to program the status register */
        BYTECOUNT   number               /* number of register to program ,1~3 Status Register0~3*/
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    if (number < 1 || number > 3)return SLLD_ERROR;
    switch(number)
    {
    case 1:
        status = FLASH_WR(device_num,SPI_WRR_CMD, ADDRESS_NOT_USED, status_val, 1);
        break;
    case 2:
        status = FLASH_WR(device_num,SPI_WRR_CMD, ADDRESS_NOT_USED, status_val, 2);
        break;
    case 3:
        status = FLASH_WR(device_num,SPI_WRR_CMD, ADDRESS_NOT_USED, status_val, 3);
        break;
    default:
        break;
    }
    return(status);
}

/******************************************************************************
 *
 * slld_WRSRegOp - Write to Status Registers operation
 *
 * This function issues the Write Registers command to the SPI Flash.
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR or SLLD_ERROR
 *
 */
SLLD_STATUS slld_WRSRegOp
(
        BYTE  device_num,            /* device number */
        BYTE        *status_val,          /* variable containing data to program the status register */
        WRR_mode    mode,                 /*WRR_mode ,VOLATILE_WR NON_VOLATILE_WR*/
        BYTECOUNT   number,               /* number of register to program ,1~3 Status Register0~3*/
        DEVSTATUS   *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;
    if(mode == VOLATILE_WR)
    {
        status = slld_WriteVolatileCmd(device_num);
    }
    else
    {
        status = slld_WRENCmd(device_num);
    }
    if(status != SLLD_OK)
        return(status);

    status = slld_WRSRegCmd(device_num, status_val, number);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case WRROp is operated on protected area */

    return(status);
}
#endif /* INCL_WRSRegCmd */

/******************************************************************************
 *
 * slld_GetDevNumFromAddr -help user to get device number from address when target device with multiple chip select
 *
 * sys_addr       device address given by system, sys_addr may includes system base address + device offset address
 * device_num   device number based on sys_addr
 *     device_num = 0x00, drive CS[0]
 *     device_num = 0x01, drive CS[1]
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_GetDevNumFromAddr
(
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE  *device_num                        /* device number */
)
{
    SLLD_STATUS status = SLLD_OK;

    *device_num = (sys_addr & BASE_ADDR_MASK) >> BASE_ADDR_SHIFT_BIT;
    
    return status;
}

#ifdef INCL_RDCR2Cmd
/******************************************************************************
 *
 * slld_RDCR2Cmd - Read from the configuration register -2
 *
 * This function issues the RDCR2 command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDCR2Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                   /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RDCR2_CMD, ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_RDCR2Cmd */

#ifdef INCL_RDCR3Cmd
/******************************************************************************
 *
 * slld_RDCR3Cmd - Read from the configuration register -3
 *
 * This function issues the RDCR3 command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RDCR3Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                   /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_RDCR3_CMD, ADDRESS_NOT_USED, target, 1);

    return(status);
}
#endif /* INCL_RDCR3Cmd */

#ifdef INCL_IRPRDCmd
/******************************************************************************
 *
 * slld_RASPCmd - Read IRP Register
 *
 * This function issues the IRPRD command to SPI Flash and reads from IRP register.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IRPRDCmd
(
        BYTE      device_num,                     //device number
        WORD    *target                        /* variable in which to store read data */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_IRPRD_CMD, (ADDRESS)ADDRESS_NOT_USED, (BYTE *)target, 2);

    return(status);
}
#endif /* INCL_IRPRDCmd */

#ifdef INCL_IRPPCmd
/******************************************************************************
 *
 * slld_IRPPCmd - Write to IRP Register
 *
 * This function issues the Write IRP register command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IRPPCmd
(
        BYTE  device_num,            /* device number */
        WORD    *irp_val                 /* variable containing data to program to the IRP register */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_IRPP_CMD, ADDRESS_NOT_USED, (BYTE *)irp_val, 2);

    return(status);
}

/******************************************************************************
 *
 * slld_IRPPOP - Write to IRP Register operation
 *
 * This function issues the Write IRP register command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IRPPOp
(
        BYTE  device_num,            /* device number */
        WORD       *irp_val,             /* variable containing data to program to the IRP register */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_IRPPCmd(device_num, irp_val);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);       /* just in case IRPPOp is operated on protected area */

    return(status);
}
#endif /* INCL_IRPPCmd */

#ifdef INCL_QPIENCmd
/******************************************************************************
 *
 * slld_QPIENCmd - Enter QPI
 *
 * This function issues the QPIEN command to enter the QPI mode.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPIENCmd
(
        BYTE  device_num            /* device number */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_QPIEN_CMD, ADDRESS_NOT_USED, 0, 0);

    return(status);
}
#endif /* INCL_QPIENCmd */

#ifdef INCL_QPIEXCmd
/******************************************************************************
 *
 * slld_QPIEXCmd - Exit QPI
 *
 * This function issues the QPIEX command to exit the QPI mode.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_QPIEXCmd
(
        BYTE  device_num            /* device number */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_QPIEX_CMD, ADDRESS_NOT_USED, 0, 0);

    return(status);
}
#endif /* INCL_QPIEXCmd */

#ifdef INCL_4BENCmd
/******************************************************************************
 *
 * slld_4BENCmd -enter 4-byte Address Mode command sets the volatile Address Length bit (CR2V[0]) to 1 to
 * change most 3-byte address commands to require 4 bytes of address.
 *
 * This function issues the enter 4-byte Address Mode command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4BENCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4BEN_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_4BEN_Cmd */

#ifdef INCL_4BEXCmd
/******************************************************************************
 *
 * slld_4BEXCmd -exit 4-byte Address Mode command sets the volatile Address Length bit (CR2V[0]) to 0.
 *
 * This function issues the exit 4-byte Address Mode command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4BEXCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4BEX_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_4BEX_Cmd */

#ifdef INCL_SECRRCmd 
/******************************************************************************
 *
 * slld_SECRRCmd - Security Region read
 *
 * This function issues the Security Region read command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_SECRRCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                      // device address given by system
        BYTE     *read_buf,                      // data buffer
        BYTECOUNT len_in_bytes                   // number of bytes
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_READ(device_num,SPI_SECRR_CMD, sys_addr, read_buf, len_in_bytes);
    return(status);
}
#endif /* INCL_SECRRCmd */

#ifdef INCL_SECRPCmd 
/******************************************************************************
 *
 * slld_SECRPCmd - Security Region program
 *
 * This function issues the Security Region program command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_SECRPCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                 // device address given by system
        BYTE     *program_buf,              // data buffer
        BYTECOUNT len_in_bytes              // number of bytes
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SECRP_CMD, sys_addr, program_buf, len_in_bytes);
    return(status);
}

/******************************************************************************
 *
 * slld_SECRPOp - Security Region program operation
 *
 * This function issues the SECRP command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_SECRPOp
(
        BYTE      device_num,                     //device number
        ADDRESS          sys_addr,                           // device address given by system
        BYTE             *program_buf,                       // data buffer
        BYTECOUNT        len_in_bytes,                       // number of bytes
        DEVSTATUS        *dev_status_ptr                     // variable to store device status
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_SECRPCmd(device_num, sys_addr, program_buf, len_in_bytes);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        // just in case SEOp is operated on a protected area

    return(status);
}
#endif /* INCL_SECRPCmd */

#ifdef INCL_SECRECmd 
/******************************************************************************
 *
 * slld_SECRECmd - Security Region erase
 *
 * This function issues the Security Region erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_SECRECmd
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr                  // device address given by system
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SECRE_CMD, sys_addr, BUFFER_NOT_USED, 0);
    return(status);
}

/******************************************************************************
 *
 * slld_SECREOp - Security Region erase operation
 *
 * This function issues the SECRE command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 */
SLLD_STATUS slld_SECREOp
(
        BYTE            device_num,                     //device number
        ADDRESS     sys_addr,                 // device address given by system
        DEVSTATUS  *dev_status_ptr            // variable to store device status
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_SECRECmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);        // just in case SEOp is operated on a protected area

    return(status);
}
#endif /* INCL_SECRECmd */

#ifdef INCL_RUIDCmd
/******************************************************************************
 *
 * slld_RUIDCmd - Read Unique ID Number
 *
 * This function issues the Read Unique ID command to SPI Flash and read 64-bit Unique Serial Number
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_RUIDCmd
(
        BYTE      device_num,                     //device number
        BYTE      *read_buf        // data buffer
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    // read 64-bit number that is unique
    status = FLASH_RD(device_num,SPI_RUID_CMD, ADDRESS_NOT_USED, read_buf, 8);

    return(status);
}
#endif /* INCL_RUIDCmd */

#ifdef INCL_WRENVCmd
/******************************************************************************
 *
 * slld_WRENVCmd - Write Enable for Volatile Status and Configure Register
 *
 * This function issues the WRENV command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_WRENVCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_WRENV_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_WRENVCmd */

#ifdef INCL_IBLRDCmd
/******************************************************************************
 *
 * slld_IBLRDCmd - IBL read.
 *
 * This function issues the reading the state of each IBL bit protection.
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IBLRDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,               /* device address given by system */
        BYTE                *target           /*variable in which to store read data*/
)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_RD(device_num,SPI_IBLRD_CMD, sys_addr, target, 1);

    return(status);
}
#endif /* INCL_IBLRDCmd */

#ifdef INCL_4IBLRDCmd
/******************************************************************************
 *
 * slld_4IBLRDCmd - IBL read - 4 Bytes address.
 *
 * This function issues the reading the state of each IBL bit protection.
 *
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4IBLRDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,               /* device address given by system */
        BYTE                *target           /*variable in which to store read data*/
)
{
    SLLD_STATUS status = SLLD_OK;
#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif
    status = FLASH_RD(device_num,SPI_4IBLRD_CMD, sys_addr, target, 1);

    return(status);
}
#endif /* INCL_4IBLRDCmd */

#ifdef INCL_IBLCmd
/******************************************************************************
 *
 * slld_IBLCmd - IBL lock
 *
 * This function issues the IBL lock command to sets the selected IBL bit to "0" protecting each related 
 * sector / block.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IBLCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_IBL_CMD, sys_addr, 0, 0);

    return(status);
}
#endif /* INCL_IBLCmd */

#ifdef INCL_4IBLCmd
/******************************************************************************
 *
 * slld_4IBLCmd - IBL lock -4 Bytes address
 *
 * This function issues the IBL lock command to sets the selected IBL bit to "0" protecting each related 
 * sector / block.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4IBLCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4IBL_CMD, sys_addr, 0, 0);

    return(status);
}
#endif /* INCL_4IBLCmd */

#ifdef INCL_IBLOp
/******************************************************************************
 *
 * slld_IBLOP - IBL lock operation
 *
 * This function issues the IBL lock command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IBLOp
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,               /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifndef USE_4B_ADDR_CMDS
    status = slld_IBLCmd(device_num, sys_addr);
#else
    status = slld_4IBLCmd(device_num, sys_addr);
#endif
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    return(status);
}
#endif /* INCL_IBLOp */

#ifdef INCL_IBULCmd
/******************************************************************************
 *
 * slld_IBULCmd - IBL unlock
 *
 * This function issues the IBL unlock command to sets the selected IBL bit to "1" unprotecting each related 
 * sector / block.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IBULCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_IBUL_CMD, sys_addr, 0, 0);

    return(status);
}
#endif /* INCL_IBULCmd */

#ifdef INCL_4IBULCmd
/******************************************************************************
 *
 * slld_4IBLCmd - IBL unlock -4 Bytes address
 *
 * This function issues the IBL unlock command to sets the selected IBL bit to "1" unprotecting each related 
 * sector / block.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4IBULCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4IBUL_CMD, sys_addr, 0, 0);

    return(status);
}
#endif /* INCL_4IBULCmd */

#ifdef INCL_IBULOp
/******************************************************************************
 *
 * slld_IBULOP - IBL unlock operation
 *
 * This function issues the IBL unlock command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_IBULOp
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,               /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifndef USE_4B_ADDR_CMDS
    status = slld_IBULCmd(device_num, sys_addr);
#else
    status = slld_4IBULCmd(device_num, sys_addr);
#endif
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    return(status);
}
#endif /* INCL_IBULOp */

#ifdef INCL_GBLCmd
/******************************************************************************
 *
 * slld_GBLCmd - Global IBL lock
 *
 * This function issues the global IBL lock command to sets all IBL bit to "0" protecting all sectors / blocks.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_GBLCmd
(
        BYTE  device_num            /* device number */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_GBL_CMD, ADDRESS_NOT_USED, 0, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_GBLOP - Global IBL lock operation
 *
 * This function issues the global IBL lock command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_GBLOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_GBLCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    return(status);
}
#endif /* INCL_GBLCmd */

#ifdef INCL_GBULCmd
/******************************************************************************
 *
 * slld_GBULCmd - Global IBL unlock
 *
 * This function issues the global IBL unlock command to sets all IBL bit to "1" unprotecting all sectors / blocks.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_GBULCmd
(
        BYTE  device_num            /* device number */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_GBUL_CMD, ADDRESS_NOT_USED, 0, 0);

    return(status);
}

/******************************************************************************
 *
 * slld_GBULOP - Global IBL unlock operation
 *
 * This function issues the global IBL unlock command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_GBULOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_GBULCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    return(status);
}
#endif /* INCL_GBULCmd */

#ifdef INCL_SPRPCmd
/******************************************************************************
 *
 * slld_SPRPCmd - set pointer region protection
 *
 * This function issues the SPRP command to sets pointer region protection.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SPRPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SPRP_CMD, sys_addr, 0, 0);

    return(status);
}
#endif /* INCL_SPRPCmd */

#ifdef INCL_4SPRPCmd
/******************************************************************************
 *
 * slld_4SPRPCmd - set pointer region protection - 4 Bytes address
 *
 * This function issues the 4SPRP command to sets pointer region protection.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4SPRPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr               /* device address given by system */
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_4SPRP_CMD, sys_addr, 0, 0);

    return(status);
}
#endif /* INCL_4SPRPCmd */

#ifdef INCL_SPRPOp
/******************************************************************************
 *
 * slld_SPRPOp - set pointer region protection operation
 *
 * This function issues the SPRP command to the SPI Flash then polls for completion.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SPRPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,               /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
)
{
    SLLD_STATUS status = SLLD_OK;

    status = slld_WRENCmd(device_num);
    if(status != SLLD_OK)
        return(status);

    status = slld_4SPRPCmd(device_num, sys_addr);
    if(status != SLLD_OK)
        return(status);

    status = slld_Poll(device_num, dev_status_ptr);
    if(status != SLLD_OK)
        return(status);

    status = slld_WRDICmd(device_num);
    return(status);
}
#endif /* INCL_SPRPOp */

#ifdef INCL_PRLCmd
/******************************************************************************
 *
 * slld_PRLCmd - Protection register lock (NVLOCK bit write)
 *
 * This function issues the PRL command to SPI Flash
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PRLCmd(BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_PRL_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0);

    return(status);
}
#endif /* INCL_PRLCmd */

#ifdef INCL_PRRDCmd
/******************************************************************************
 *
 * slld_PRRDCmd - Protection register Read
 *
 * This function issues the PRRD command to the SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_PRRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf                  /* variable containing data to program */
)
{
    SLLD_STATUS         status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_RD(device_num,SPI_PRRD_CMD, ADDRESS_NOT_USED, data_buf, 1);

    return(status);
}
#endif /* INCL_PRRDCmd */

#ifdef INCL_SPI_SETBUSRTLENGTH
/******************************************************************************
 *
 * slld_BSLCmd - Set Burst Length
 *
 * This function issues the Set Burst Length command to SPI Flash.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_SBLCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *wrapbit_buf                     // variable in which to store wrapbit data
)
{
    SLLD_STATUS status = SLLD_OK;
    BYTE buf[4]={0xff,0xff,0xff,0xff};
    
    buf[3] = *wrapbit_buf;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_SETBURSTWRAP_CMD, ADDRESS_NOT_USED, buf, 4);

    return(status);
}
#endif

#ifdef  INCL_SPI_MODEBITRESET
/******************************************************************************
 *
 * slld_MBRCmd - Mode Bit Reset
 *
 * This function issues MBR command toreturn the device from continuous high performance read mode 
 * back to normal standby awaiting any new command.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_MBRCmd (BYTE device_num)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    status = FLASH_WR(device_num,SPI_MBR_CMD, ADDRESS_NOT_USED, 0, 0);

    return(status);
}
#endif /* INCL_SPI_MODEBITRESET */

#ifdef INCL_DDRDIORCmd
/******************************************************************************
 *
 * slld_DDRDIORCmd - Read flash using DDR dual IO
 *
 * This function issues the DDR dual IO read command and reads the requested data.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_DDRDIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_DDRDIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_DDRDIORCmd */

#ifdef INCL_4DDRDIORCmd
/******************************************************************************
 *
 * slld_4DDRDIORCmd - Read flash using DDR dual IO in 4-bytes addressing scheme
 *
 * This function issues the DDR dual IO read command and reads the requested data in 4-bytes
 * addressing scheme.
 *
 * RETURNS: SLLD_OK, SLLD_E_DEVICE_SOFTWARE_PROTECTED or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS slld_4DDRDIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
)
{
    SLLD_STATUS status = SLLD_OK;

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
    /* check if target device is software protected */
    if(sys_software_protect_status != FLASH_SOFTWARE_UNPROTECTED)
    {
        status = SLLD_E_DEVICE_SOFTWARE_PROTECTED;
        return(status);
    }
#endif

    modebit_char = modebit;
    status = FLASH_RD(device_num,SPI_4DDRDIOR_CMD, sys_addr, target, len_in_bytes);

    return(status);
}
#endif /* INCL_4DDRDIORCmd */
