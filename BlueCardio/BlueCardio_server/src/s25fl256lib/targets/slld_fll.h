/* slld_fld.h - Device configuration file for the SLLD */

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

#ifndef  __INC_slldtargetspecifich
#define __INC_slldtargetspecifich

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#undef INCL_ReadSecurityCmd
#undef INCL_ProgramSecurityCmd
#undef INCL_EraseSecurityCmd
#undef INCL_ReadIdDualCmd
#undef INCL_ReadIdQuadCmd
#undef INCL_BE32KBCmd
#undef INCL_WordReadQuadCmd
#undef INCL_ReadUniqueIDCmd
#undef INCL_OctalWordReadQuadCmd
#undef INCL_WriteVolatileCmd
#define INCL_Read_IDCmd
#define INCL_RDIDCmd
#undef INCL_READ_IDENTIFICATIONCmd
#define INCL_RDSRCmd
#undef INCL_RASPCmd
#undef INCL_BRRDCmd
#undef INCL_ABRDCmd
#undef INCL_ECCRDCmd
#undef INCL_RPWDCmd
#define INCL_WRENCmd
#define INCL_WRDICmd
#undef INCL_WRSRCmd
#undef INCL_SRSTCmd
#undef INCL_PPB_PGCmd
#undef INCL_DYB_PGCmd
#undef INCL_ERS_SSPCmd
#undef INCL_ERS_RESCmd
#undef INCL_RCSPCmd
#undef INCL_RCRSCmd
#undef INCL_RCVRCmd
#define INCL_DPCmd
#undef INCL_BRACCCmd
#undef INCL_SPCmd
#define INCL_RESCmd
#define INCL_CLSRCmd
#define INCL_WRRCmd
#undef INCL_WASPCmd
#undef INCL_BRWRCmd
#undef INCL_ABWRCmd
#undef INCL_WPWDCmd
#define INCL_RCRCmd
#undef INCL_OTPRCmd
#undef INCL_OTPPCmd
#define INCL_ReadCmd
#define INCL_Read_4BCmd
#define INCL_Fast_ReadCmd
#define INCL_Fast_Read_4BCmd
#undef INCL_DUALOUTPUT_READCmd
#undef INCL_DUALOUTPUT_READ_4BCmd
#define INCL_DORCmd
#define INCL_4DORCmd
#undef INCL_QUADOUTPUT_READCmd
#undef INCL_QUADOUTPUT_READ_4BCmd
#define INCL_QORCmd
#define INCL_4QORCmd
#undef INCL_DUALOUTPUT_HP_READCmd
#undef INCL_DUALOUTPUT_HP_READ_4BCmd
#define INCL_DIORCmd
#define INCL_4DIORCmd
#undef INCL_QUADOUTPUT_HP_READCmd
#undef INCL_QUADOUTPUT_HP_READ_4BCmd
#define INCL_QIORCmd
#define INCL_4QIORCmd
#undef INCL_DDR_Fast_ReadCmd
#undef INCL_DDR_Fast_4BReadCmd
#undef INCL_DDR_DUALOUTPUT_HP_READCmd
#undef INCL_DDR_DUALOUTPUT_HP_READ_4BCmd
#undef INCL_DDRDIORCmd
#undef INCL_4DDRDIORCmd
#undef INCL_DDR_QUADOUTPUT_HP_READCmd
#undef INCL_DDR_QUADOUTPUT_HP_READ_4BCmd
#define INCL_DDRQIORCmd
#define INCL_4DDRQIORCmd
#undef INCL_BulkEraseCmd
#define INCL_SECmd
#define INCL_SE_4BCmd
#define INCL_HBECmd
#define INCL_4HBECmd
#define INCL_4BECmd
#define INCL_CECmd
#define INCL_CE1Cmd
#define INCL_BLOCKERASECmd
#define INCL_PPCmd
#define INCL_PP_4BCmd
#undef INCL_P4ECmd
#undef INCL_P4E_4BCmd
#undef INCL_P8ECmd
#undef INCL_P8E_4BCmd
#define INCL_QPPCmd
#define INCL_QPP_4BCmd
#undef INCL_QPP2md
#undef INCL_MPMCmd
#define INCL_BufferedProgramOp
#define INCL_BufferedProgram_4BOp
#define INCL_BlockProtectOp
#define INCL_QuadWriteOp
#define INCL_RDSR2Cmd
#define INCL_DLPRDCmd
#define INCL_PNVDLRCmd
#define INCL_WVDLRCmd
#undef INCL_PGSPCmd 
#undef INCL_PGRSCmd 
#undef INCL_PLBWRCmd
#undef INCL_PLBRDCmd
#undef INCL_DYB_RDCmd
#undef INCL_PPB_RDCmd
#undef INCL_PPB_ERSCmd
#define INCL_PASSRDCmd
#define INCL_PASSPCmd
#define INCL_PASSUCmd
#define INCL_RDARCmd
#define INCL_WRARCmd
#define INCL_RSTENCmd
#define INCL_RSTCmd
#define INCL_RDQIDCmd
#undef INCL_EESCmd
#undef INCL_DYBRD_Cmd
#undef INCL_PPBRD_Cmd
#undef INCL_PPBP_Cmd
#undef INCL_DYBWR_Cmd
#define INCL_EPR_Cmd
#define INCL_EPS_Cmd
#undef INCL_4BAM_Cmd
#define INCL_SPI_READSFDPCMD
#undef INCL_SPI_READMODERESETCMD
#undef INCL_SPI_BURSTWRAPCMD
#undef INCL_RDSR3Cmd
#undef INCL_WRSRegCmd
#define INCL_RDCR2Cmd
#define INCL_RDCR3Cmd
#define INCL_IRPRDCmd
#define INCL_IRPPCmd
#define INCL_QPIENCmd
#define INCL_QPIEXCmd
#define INCL_4BENCmd
#define INCL_4BEXCmd
#define INCL_SECRRCmd
#define INCL_SECRPCmd
#define INCL_SECRECmd
#define INCL_RUIDCmd
#define INCL_WRENVCmd
#define INCL_IBLRDCmd
#define INCL_4IBLRDCmd
#define INCL_IBLCmd
#define INCL_4IBLCmd
#define INCL_IBLOp
#define INCL_IBULCmd
#define INCL_4IBULCmd
#define INCL_IBULOp
#define INCL_GBLCmd
#define INCL_GBULCmd
#define INCL_SPRPCmd
#define INCL_4SPRPCmd
#define INCL_SPRPOp
#define INCL_PRLCmd
#define INCL_PRRDCmd
#define INCL_SPI_SETBUSRTLENGTH
#define INCL_SPI_MODEBITRESET

// for function internal
#define STATUS_IN_SR2 
#define BLOCK_PROTECT_USE_WRAR 
#undef BURSTWRAP_SIZE_1 
#undef USE_4B_ADDR_BAR

// for command instruction
#undef CMD_RDSR2_35 
#undef CMD_CLSR_82 
#define CMD_EPR_7A	
#define CMD_EPS_75 
#define CMD_SBL_77 
#define CMD_SE_20  
#define CMD_4SE_21 
#define CMD_PASSU_EA 
//-----------------------------------------------------------

// don't need to sunifdef
//INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
//USE_4B_ADDR_CMDS

// Flash page size
#define PAGE_SIZE (256)
#define PAGE_MASK (PAGE_SIZE-1)

// Status register error bits definition
#define STATUS_ERROR_FLAGS  (HAS_STATBIT5_ERROR | HAS_STATBIT6_ERROR)

// Read ID command
#define SPI_READ_ID_CMD SPI_RDID_AB_CMD
#define SPI_RDID_CMD SPI_RDID_9F_CMD

// defin block protect bit mask
#define BLOCK_PROTECT_BITS_MASK  (0x1C)

//************************************************************************
// enables code to execute commands from a file instead of directly from *
// the command line.													 *
//************************************************************************
#define ENABLE_SCRIPTING_MACRO

/************************************************************************
 * Creates lld_printf.log file and writes all the LLD_PRINTFs to it      *
 ************************************************************************/
//#define LLD_PRINTF_LOGFILE


//*******************************************************
// Define system base address   *
// Followed is just example code, user have to modify the mask and shift bit number based on own system configuration
//*******************************************************
#define BASE_ADDR_MASK    0xC0000000
#define BASE_ADDR_SHIFT_BIT 30
//*******************************************************
// Define flash simulator R/W debug macro (no trace)    *
//*******************************************************
#ifdef __FLASHMODEL
// The number of clocks to write various types of data
#define WRITEDATALEN 8
#define WRITECMDLEN  8
#define WRITEADDR24  24
#define WRITEADDR32  32

#define DEBUG_FLASH_WR(b,d)   spiSignalManagerAdapter->SM_ADPT_WR((UINT32)(b), WRITEDATA, d, WRITEDATALEN)
#define DEBUG_FLASH_RD(b)     spiSignalManagerAdapter->SM_ADPT_RD()
#else
// The number of clocks to write various types of data
// these values are only used by the model
#define WRITEDATALEN 0
#define WRITECMDLEN  0
#define WRITEADDR24  0
#define WRITEADDR32  0

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __INC_slldtargetspecifich */
