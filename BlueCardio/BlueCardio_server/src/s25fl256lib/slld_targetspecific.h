/* slld_targetspecific.h - Device configuration file for the SLLD */

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

//#ifndef  __INC_slldtargetspecifich
//#define __INC_slldtargetspecifich

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Turn software trace on/off
//#define TRACE
#include "slld_fls_127s_256wb.h"
#if 0
/**********************************************************
 * Define LLD target device: flash.                        *
 ***********************************************************/
#define SLLD_DEV_FLASH
//define SLLD_DEV_SIM

//
// Uncomment the line that corresponds to the product type you want to use
//

// #define FL_D
// #define FL_A
// #define FL_P_SINGLE_BIT_IO
// #define FL_P_MULTI_BIT_IO
// #define FL_R
// #define FL_T
 #define FL_127S_256_WRITE_BUFFER
// #define FL_127S_512_WRITE_BUFFER
// #define FS_S_256_WRITE_BUFFER
// #define FS_S_512_WRITE_BUFFER
// #define FL_S
//#define FL_L
// #define FL_K
// #define FL_2K
// #define FL_1K
/* Enable 4 byte address commands in slld_ReadOp, slld_WriteOp, and slld_SEOp */
// #define USE_4B_ADDR_CMDS

#ifdef FL_L

// #define FL128L
#define FL256L
//#define FL512L
//#define FL512L_DDP
// #define FL256L_DQ
// #define FL512L_DQ

#if !( defined(FL256L) || defined (FL128L) || defined (FL512L) || defined (FL512L_DDP) || defined (FL256L_DQ) || defined (FL512L_DQ))
#error "must define a sub device for FLL serial"
#endif
#endif

/* Enable 4 byte address via Bank Address Register in slld_ReadOp, slld_WriteOp, and slld_SEOp */
//#define USE_4B_ADDR_BAR
/* Uncomment next line to enable software protect check in each function */
/* #define INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK    */

/* If you are worried about code size, you can      */
/* remove an LLD function by un-commenting the      */
/* appropriate macro below.                         */
/*                                                  */
/* #define REMOVE_PPOp                              */
/* #define REMOVE_PP_4BOp                           */
/* #define REMOVE_BufferedProgramOp                 */
/* #define REMOVE_SEOp                              */
/* #define REMOVE_SE_4BOp                           */
/* #define REMOVE_BulkEraseOp                              */
/* #define REMOVE_WRSROp                            */
/* #define REMOVE_BlockProtectOp                    */
/* #define REMOVE_ReadCmd                           */
/* #define REMOVE_Read_4BCmd                        */
/* #define REMOVE_Fast_ReadCmd                      */
/* #define REMOVE_Fast_Read_4BCmd                   */
/* #define REMOVE_Read_IDCmd                        */
/* #define REMOVE_RDIDCmd                           */
/* #define REMOVE_SRSTCmd                           */
/* #define REMOVE_RDSRCmd                           */
/* #define REMOVE_RASPCmd                           */
/* #define REMOVE_RBNKCmd                           */
/* #define REMOVE_RABTCmd                           */
/* #define REMOVE_RECCCmd                           */
/* #define REMOVE_RPWDCmd                           */
/* #define REMOVE_WRENCmd                           */
/* #define REMOVE_WRDICmd                           */
/* #define REMOVE_WRSRCmd                           */
/* #define REMOVE_PPCmd                             */
/* #define REMOVE_PP_4BCmd                          */
/* #define REMOVE_SECmd                             */
/* #define REMOVE_SE_4BCmd                          */
/* #define REMOVE_BulkEraseCmd                             */
/* #define REMOVE_SPCmd                             */
/* #define REMOVE_RESCmd                            */

/* #define REMOVE_PPB_PGCmd                         */
/* #define REMOVE_PPB_PGOp                          */
/* #define REMOVE_DYB_PGCmd                         */
/* #define REMOVE_DYB_PGOp                          */
/* #define REMOVE_ERS_SSPCmd                        */
/* #define REMOVE_ERS_RESCmd                        */
/* #define REMOVE_RCSPCmd                           */
/* #define REMOVE_RCRSCmd                           */
/* #define REMOVE_RCVRCmd                           */

/*  slld_poll and slld_StatusGet utility functions are 
 *  used in Operation function, like slld_PPOp, so should
 *  be included when Operation functions are used.
 */

/* #define REMOVE_Poll                              */
/* #define REMOVE_StatusGet                         */
/* #define REMOVE_SoftwareProtectStatusGet          */

/*  The folowing Macros will remove the new functions
 *  that are required for dual and quad mode SPI devices
 */

/* #define REMOVE_MULTIOMODE_FUNCTIONS              */
/* #define REMOVE_READ_IDENTIFICATIONCmd            */
/* #define REMOVE_READ_IDCFI                        */
/* #define REMOVE_CLSRCmd                           */
/* #define REMOVE_WRRCmd                            */
/* #define REMOVE_WRROp                             */
/* #define REMOVE_WASPCmd                           */
/* #define REMOVE_WASPOp                            */
/* #define REMOVE_WBNKCmd                           */
/* #define REMOVE_WBNKOp                            */
/* #define REMOVE_WABTCmd                           */
/* #define REMOVE_WABTOp                            */
/* #define REMOVE_WPWDCmd                           */
/* #define REMOVE_WPWDOp                            */
/* #define REMOVE_RCRCmd                            */
/* #define REMOVE_DUALOUTPUT_READCmd                */
/* #define REMOVE_DUALOUTPUT_READ_4BCmd             */
/* #define REMOVE_QUADOUTPUT_READ                   */
/* #define REMOVE_QUADOUTPUT_READ_4BCmd             */
/* #define REMOVE_DUALOUTPUT_HP_READ                */
/* #define REMOVE_DUALOUTPUT_HP_READ_4BCmd          */
/* #define REMOVE_QUADOUTPUT_HP_READ                */
/* #define REMOVE_QUADOUTPUT_HP_READ_4BCmd          */
/* #define REMOVE_QUAD_PAGE_PROGRAMCmd              */
/* #define REMOVE_QUAD_PAGE_PROGRAM_4BCmd           */
/* #define REMOVE_QUAD_PAGE_PROGRAMOp               */
/* #define REMOVE_QUAD_PAGE_PROGRAM_4BOp            */
/* #define REMOVE_P4EOp                             */
/* #define REMOVE_P8EOp                             */
/* #define REMOVE_P8E_4BOp                          */
/* #define REMOVE_P4ECmd                            */
/* #define REMOVE_P8ECmd                            */
/* #define REMOVE_P8E_4BCmd                         */
/* #define REMOVE_OTPPCmd                           */
/* #define REMOVE_OTPPOp                            */
/* #define REMOVE_OTPRCmd                           */
/* #define REMOVE_MPMCmd                            */

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
#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */
//#endif /* __INC_slldtargetspecifich */
