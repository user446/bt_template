/* slld.h - Header file for Cypress SPI Low Level Driver */

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

#ifndef __INC_lldh
#define __INC_lldh

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define SLLD_VERSION          "16.2.1"

// ==================
// Status error flags
// ==================
#define HAS_NOTHING          0x00
#define HAS_STATBIT5_ERROR   0x01
#define HAS_STATBIT6_ERROR   0x02



#include "slld_targetspecific.h"

// =======================
// SPI Flash Commands info
// =======================
#ifdef INCL_WRSRCmd
#define SPI_WRSR_CMD                (0x01)
#endif
#if defined (INCL_WRRCmd) || defined (INCL_WRSRegCmd)
#define SPI_WRR_CMD                 (0x01)
#endif
#ifdef INCL_PPCmd
#define SPI_PP_CMD                  (0x02)
#endif
#ifdef INCL_ReadCmd
#define SPI_READ_CMD                (0x03)
#endif
#ifdef INCL_WRDICmd
#define SPI_WRDI_CMD                (0x04)
#endif
#ifdef INCL_RDSRCmd
#define SPI_RDSR_CMD                (0x05)
#endif
#ifdef INCL_WRENCmd
#define SPI_WREN_CMD                (0x06)
#endif 
#ifdef INCL_RDSR2Cmd
#ifdef CMD_RDSR2_35
#define SPI_RDSR2_CMD               (0x35)  // Read Status Register-2
#else
#define SPI_RDSR2_CMD               (0x07)  // Read Status Register-2
#endif
#endif
#ifdef INCL_Fast_ReadCmd
#define SPI_FAST_READ_CMD           (0x0B)
#endif
#ifdef INCL_Fast_Read_4BCmd
#define SPI_FAST_READ_4B_CMD        (0x0C)
#endif
#ifdef INCL_DDR_Fast_ReadCmd
#define SPI_FAST_READ_DDR_CMD       (0x0D)
#endif
#ifdef INCL_DDR_Fast_4BReadCmd
#define SPI_FAST_READ_DDR_4B_CMD    (0x0E)
#endif
#ifdef INCL_PP_4BCmd
#define SPI_PP_4B_CMD               (0x12)
#endif
#ifdef INCL_Read_4BCmd
#define SPI_READ_4B_CMD             (0x13)
#endif
#ifdef INCL_ABRDCmd
#define SPI_ABRD_CMD                (0x14)  // AutoBoot Register Read
#endif
#ifdef INCL_RDCR2Cmd
#define SPI_RDCR2_CMD               (0x15) // Read configuration register-2
#endif
#ifdef INCL_ABWRCmd
#define SPI_ABWR_CMD                (0x15)  // AutoBoot Register Write
#endif
#ifdef INCL_BRRDCmd
#define SPI_BRRD_CMD                (0x16)  // Bank Register Read
#endif
#ifdef INCL_BRWRCmd
#define SPI_BRWR_CMD                (0x17)  // Bank Register Write
#endif
#ifdef INCL_ECCRDCmd
#define SPI_ECCRD_CMD               (0x18)  // ECC Read
#endif
#ifdef INCL_P4ECmd
#define SPI_P4E_CMD                 (0x20)
#endif
#ifdef INCL_P4E_4BCmd
#define SPI_P4E4_CMD                (0x21)  // Parameter 4K-sector Erase (4Byte Addr)
#endif
#ifdef INCL_RASPCmd
#define SPI_RASP_CMD                (0x2B) 
#endif
#ifdef INCL_IRPRDCmd
#define SPI_IRPRD_CMD               (0x2B) // IRP register Read
#endif
#ifdef INCL_WASPCmd
#define SPI_WASP_CMD                (0x2F)
#endif
#ifdef INCL_IRPPCmd
#define SPI_IRPP_CMD                  (0x2F) // IRP register program
#endif
#ifdef INCL_CLSRCmd
#ifdef CMD_CLSR_82
#define SPI_CLSR_CMD                (0x82)
#else
#define SPI_CLSR_CMD                (0x30)
#endif
#endif
#ifdef INCL_QPPCmd
#define SPI_QPP_CMD                 (0x32)
#endif
#ifdef INCL_RDSR3Cmd
#define SPI_RDSR3_CMD               (0x33)  // Read Status Register-3
#endif
#ifdef INCL_RDCR3Cmd
#define SPI_RDCR3_CMD           (0x33) // Read Configuration Register-3
#endif
#ifdef INCL_QPP_4BCmd
#define SPI_QPP_4B_CMD              (0x34)
#endif
#ifdef INCL_RCRCmd
#define SPI_RCR_CMD                 (0x35)
#endif
#ifdef INCL_IBLCmd
#define SPI_IBL_CMD                   (0x36) // IBL lock
#endif
#ifdef INCL_QPP2md
#define SPI_QPP2_CMD                (0x38)  // Quad Page Program (3Byte Addr)
#endif
#ifdef INCL_QPIENCmd
#define SPI_QPIEN_CMD               (0x38) // Enter QPI
#endif
#ifdef INCL_IBULCmd
#define SPI_IBUL_CMD                 (0x39) // IBL unlock
#endif
#ifdef INCL_DUALOUTPUT_READCmd
#define SPI_DUALIO_RD_CMD           (0x3B)
#endif
#ifdef INCL_DORCmd
#define SPI_DOR_CMD             (0x3B) // Dual output read
#endif
#ifdef INCL_DUALOUTPUT_READ_4BCmd
#define SPI_DUALIO_RD_4B_CMD        (0x3C)
#endif
#ifdef INCL_4DORCmd
#define SPI_4DOR_CMD              (0x3C) // Dual output read 4 Byte address 
#endif
#ifdef INCL_IBLRDCmd
#define SPI_IBLRD_CMD           (0x3D) // IBL read
#endif
#ifdef INCL_P8ECmd
#define SPI_P8E_CMD                 (0x40)
#endif
#ifdef INCL_DLPRDCmd
#define SPI_DLPRD_CMD               (0x41)  // Read Data Learning Pattern
#endif
#ifdef INCL_OTPPCmd
#define SPI_OTPP_CMD                (0x42)
#endif
#ifdef INCL_ProgramSecurityCmd
#define SPI_PROGRAM_SECURITY_CMD    (0x42)  // Program Security Register
#endif
#ifdef INCL_SECRPCmd
#define SPI_SECRP_CMD           (0x42) // security region program
#endif
#ifdef INCL_PNVDLRCmd
#define SPI_PNVDLR_CMD              (0x43)  // Program NV Data Learning Register
#endif
#ifdef INCL_EraseSecurityCmd
#define SPI_ERASE_SECURITY_CMD      (0x44)  // Erase Security Register
#endif
#ifdef INCL_SECRECmd
#define SPI_SECRE_CMD             (0x44) // security region erase
#endif
#ifdef INCL_ReadSecurityCmd
#define SPI_READ_SECURITY_CMD       (0x48)  // Read Security Register
#endif
#ifdef INCL_SECRRCmd
#define SPI_SECRR_CMD             (0x48) // security region read
#endif
#ifdef INCL_WVDLRCmd
#define SPI_WVDLR_CMD               (0x4A)  // Write Volatile Data Learning Register
#endif
#ifdef INCL_OTPRCmd
#define SPI_OTPR_CMD                (0x4B)
#endif
#ifdef INCL_ReadUniqueIDCmd
#define SPI_READ_UNIQUE_ID_CMD      (0x4B)  // Read Unique ID Number
#endif
#ifdef INCL_RUIDCmd
#define SPI_RUID_CMD                (0x4B)
#endif
#ifdef INCL_P8E_4BCmd
#define SPI_P8E_4B_CMD              (0x4C)
#endif
#ifdef INCL_WriteVolatileCmd
#define SPI_WRITE_VOLATILE_CMD      (0x50)  // Write Enable for Volatile Status Register
#endif
#ifdef INCL_WRENVCmd
#define SPI_WRENV_CMD            (0x50) // Write enable for Volatile status and configureation registers
#endif
#ifdef INCL_BE32KBCmd
#define SPI_BE32KB_CMD              (0x52)  // Block Erase 32KB
#endif
#ifdef INCL_HBECmd
#define SPI_HBE_CMD             (0x52) // Half Block Erase
#endif
#ifdef INCL_4HBECmd
#define SPI_4HBE_CMD            (0x53) // Half Block Erase 4B address
#endif
#ifdef INCL_SPI_READSFDPCMD
#define SPI_READ_SFDP_CMD           (0x5A)  // Read Serial Flash Discoverable Parameter Register
#endif
#ifdef INCL_CECmd
#define SPI_CE_CMD              (0x60) // chip erase
#endif
#ifdef INCL_BulkEraseCmd
#define SPI_BULKERASE1_CMD                 (0x60)  // Bulk Erase
#endif
#ifdef INCL_RDARCmd
#define SPI_RDAR_CMD                (0x65)  // Read Any Register
#endif
#ifdef INCL_RSTENCmd
#define SPI_RSTEN_CMD               (0x66)  // Software Reset Enable
#endif
#ifdef INCL_QUADOUTPUT_READCmd
#define SPI_QUADIO_RD_CMD           (0x6B)
#endif
#ifdef INCL_QORCmd
#define SPI_QOR_CMD                 (0x6B) // Quad Output Read
#endif
#ifdef INCL_QUADOUTPUT_READ_4BCmd
#define SPI_QUADIO_RD_4B_CMD        (0x6C)
#endif
#ifdef INCL_4QORCmd
#define SPI_4QOR_CMD                  (0x6C) // Quad Output Read 4B address
#endif
#ifdef INCL_WRARCmd
#define SPI_WRAR_CMD                (0x71)  // Write Any Register
#endif
#ifdef INCL_ERS_SSPCmd
#define SPI_ERS_SSP_CMD             (0x75)  // Erase / Program Suspend
#endif
#if defined (INCL_SPI_BURSTWRAPCMD) || defined (INCL_SPI_SETBUSRTLENGTH)
#define SPI_SETBURSTWRAP_CMD        (0x77)  // Set Burst with Wrap
#endif
#ifdef INCL_ERS_RESCmd
#define SPI_ERS_RES_CMD             (0x7A)  // Erase / Program Resume
#endif
#ifdef INCL_GBLCmd
#define SPI_GBL_CMD               (0x7E) // Global IBL lock
#endif 
//#define SPI_CLSR2_CMD               (0x82)  //Clear Status Register 1 (alternate instruction) - Erase/Prog. Fail Reset
#ifdef INCL_PGSPCmd
#define SPI_PGSP_CMD                (0x85)  // Program Suspend
#endif
#ifdef INCL_PGRSCmd
#define SPI_PGRS_CMD                (0x8A)  // Program Resume
#endif
#define SPI_RDID_90_CMD  (0x90)
#ifdef INCL_ReadIdDualCmd
#define SPI_READID_DUAL_CMD         (0x92)  // Read Device ID by Dual
#endif
#ifdef INCL_ReadIdQuadCmd
#define SPI_READID_QUAD_CMD         (0x94)  // Read Device ID by Quad
#endif
#ifdef INCL_GBULCmd
#define SPI_GBUL_CMD                (0x98) // Global IBL unlock 
#endif
#ifdef INCL_RSTCmd
#define SPI_RESET_CMD               (0x99)  // Software Reset
#endif
#define SPI_RDID_9F_CMD          (0x9F)
#ifdef INCL_MPMCmd
#define SPI_MPM_CMD                 (0xA3)
#endif
#ifdef INCL_PLBWRCmd
#define SPI_PLBWR_CMD               (0xA6)  // PPB Lock Bit Write
#endif
#ifdef INCL_PRLCmd
#define SPI_PRL_CMD                (0xA6) // Protection Register lock
#endif
#ifdef INCL_PLBRDCmd
#define SPI_PLBRD_CMD               (0xA7)  // PPB Lock Bit Read
#endif
#ifdef INCL_PRRDCmd
#define SPI_PRRD_CMD                  (0xA7) // Protection Register read
#endif
#define SPI_RDID_AB_CMD             (0xAB)
#ifdef INCL_RESCmd
#define SPI_RES_CMD                 (0xAB)
#endif
#ifdef INCL_RDQIDCmd
#define SPI_RDQID_CMD               (0xAF)  //Read Quad ID
#endif
#ifdef INCL_EPS_Cmd
#ifdef CMD_EPS_75
#define SPI_EPS_CMD                 (0x75)  // for FLL
#else
#define SPI_EPS_CMD                 (0xB0)  //Erase / Program Suspend (alternate instruction)
#endif
#endif
#ifdef INCL_EPR_Cmd
#ifdef CMD_EPR_7A
#define SPI_EPR_CMD                 (0x7A) // for FLL
#else
#define SPI_EPR_CMD                 (0x30)  //Erase / Program resume (alternate instruction)
#endif
#endif
#ifdef INCL_4BAM_Cmd
#define SPI_4BAM_CMD                (0xB7)  //Enter 4-byte Address Mode
#endif
#ifdef INCL_4BENCmd
#define SPI_4BEN_CMD                (0xB7) // Enter 4-bytes Address Mode
#endif
#ifdef INCL_SPCmd
#define SPI_SP_CMD                  (0xB9)
#endif
#ifdef INCL_DPCmd
#define SPI_DP_CMD                  (0xB9)
#endif
#ifdef INCL_BRACCCmd
#define SPI_BRAC_CMD                (0xB9)
#endif
#ifdef INCL_DUALOUTPUT_HP_READCmd
#define SPI_DUALIO_HPRD_CMD         (0xBB)
#endif
#ifdef INCL_DIORCmd
#define SPI_DIOR_CMD            (0xBB) // Dual IO read
#endif
#ifdef INCL_DUALOUTPUT_HP_READ_4BCmd
#define SPI_DUALIO_HPRD_4B_CMD      (0xBC)
#endif
#ifdef INCL_4DIORCmd
#define SPI_4DIOR_CMD          (0xBC) // Dual IO read 4B address
#endif
#ifdef INCL_DDR_DUALOUTPUT_HP_READCmd
#define SPI_DDR_DUALIO_HPRD_CMD     (0xBD)
#endif
#ifdef INCL_DDR_DUALOUTPUT_HP_READ_4BCmd
#define SPI_DDR_DUALIO_HPRD_4B_CMD  (0xBE)
#endif
#ifdef INCL_SPI_BURSTWRAPCMD
#ifdef CMD_SBL_77
#define SPI_SBL_CMD                 (0x77) // Set Burst Length
#else
#define SPI_SBL_CMD                 (0xC0)  // Set Burst Length
#endif
#endif
#ifdef INCL_CE1Cmd
#define SPI_CE1_CMD              (0xC7) // chip erase
#endif
#ifdef INCL_BulkEraseCmd
#define SPI_BULKERASE_CMD                  (0xC7)  // BULK ERASE
#endif
#ifdef INCL_EESCmd
#define SPI_EES_CMD                 (0xD0)  //Evaluate Erase Status
#endif
#ifdef INCL_SECmd
#ifdef CMD_SE_20
#define SPI_SE_CMD          (0x20)  // Sector Erase
#else
#define SPI_SE_CMD                  (0xD8)
#endif
#endif
#ifdef INCL_SE_4BCmd
#ifdef CMD_4SE_21
#define SPI_SE_4B_CMD               (0x21)
#else
#define SPI_SE_4B_CMD               (0xDC)
#endif
#endif
#ifdef INCL_BLOCKERASECmd
#define SPI_BLOCK_ERASE_CMD         (0xD8)  // BLOCK ERASE
#endif
#ifdef INCL_4BECmd
#define SPI_4BE_CMD             (0xDC) // Block erase 4B address
#endif
#ifdef INCL_DYB_RDCmd
#define SPI_DYB_RD_CMD              (0xE0)  // DYB Read  -32bit address
#endif
#ifdef INCL_4IBLRDCmd
#define SPI_4IBLRD_CMD              (0xE0) // IBL read 4B address
#endif
#ifdef INCL_DYB_PGCmd
#define SPI_DYB_PG_CMD              (0xE1)  // DYB Write -32bit address
#endif
#ifdef INCL_4IBLCmd
#define SPI_4IBL_CMD                   (0xE1) // IBL lock 4B address
#endif
#ifdef INCL_PPB_RDCmd
#define SPI_PPB_RD_CMD              (0xE2)  // PPB Read  -32bit address
#endif
#ifdef INCL_4IBULCmd
#define SPI_4IBUL_CMD                 (0xE2) // IBL unlock 4B address
#endif
#ifdef INCL_PPB_PGCmd
#define SPI_PPB_PG_CMD              (0xE3)  // PPB Write -32bit address
#endif
#ifdef INCL_4SPRPCmd
#define SPI_4SPRP_CMD               (0xE3) // Set Point Region Protection 4B address
#endif
#ifdef INCL_OctalWordReadQuadCmd
#define SPI_OCTALWORDREADQUAD_CMD   (0xE3)  // Octal Word Read Quad
#endif
#ifdef INCL_PPB_ERSCmd
#define SPI_PPB_ERS_CMD             (0xE4)  // PPB Erase
#endif
#ifdef INCL_PASSRDCmd
#define SPI_PASSRD_CMD              (0xE7)  // Password Read
#endif
#ifdef INCL_RPWDCmd
#define SPI_RPWD_CMD                (0xE7)
#endif
#ifdef INCL_WordReadQuadCmd
#define SPI_WORDREADQUAD_CMD        (0xE7)  // Word Read Quad
#endif
#ifdef INCL_PASSPCmd
#define SPI_PASSP_CMD               (0xE8)  // Password Program
#endif
#ifdef INCL_WPWDCmd
#define SPI_WPWD_CMD                (0xE8)
#endif
#ifdef INCL_4BEXCmd
#define SPI_4BEX_CMD                (0xE9) // Exit 4 Byte Address mode
#endif
#ifdef INCL_PASSUCmd
#ifdef CMD_PASSU_EA
#define SPI_PASSU_CMD            (0xEA) // Password Unlock
#else
#define SPI_PASSU_CMD               (0xE9)  // Password Unlock
#endif
#endif
#ifdef INCL_QUADOUTPUT_HP_READCmd
#define SPI_QUADIO_HPRD_CMD         (0xEB)
#endif
#ifdef INCL_QIORCmd
#define SPI_QIOR_CMD            (0xEB) // Quad IO read
#endif
#ifdef INCL_QUADOUTPUT_HP_READ_4BCmd
#define SPI_QUADIO_HPRD_4B_CMD      (0xEC)
#endif
#ifdef INCL_4QIORCmd
#define SPI_4QIOR_CMD          (0xEC) // Quad IO read 4B address
#endif
#ifdef INCL_DDR_QUADOUTPUT_HP_READCmd
#define SPI_DDR_QUADIO_HPRD_CMD     (0xED)
#endif
#ifdef INCL_DDRQIORCmd
#define SPI_DDRQIOR_CMD         (0xED) // DDR Quad IO read
#endif
#ifdef INCL_DDR_QUADOUTPUT_HP_READ_4BCmd
#define SPI_DDR_QUADIO_HPRD_4B_CMD  (0xEE)
#endif
#ifdef INCL_4DDRQIORCmd
#define SPI_4DDRQIOR_CMD       (0xEE) // DDR Quad IO read 4B address
#endif
#ifdef INCL_SRSTCmd
#define SPI_SOFTWARE_RESET          (0xF0)
#endif
#ifdef INCL_QPIEXCmd
#define SPI_QPIEX_CMD               (0xF5) // Exit QPI
#endif
#ifdef INCL_DYBRD_Cmd
#define SPI_DYBRD_CMD               (0xFA)  //DYB Read   -24bit or 32bit address
#endif
#ifdef INCL_DYBWR_Cmd
#define SPI_DYBWR_CMD               (0xFB)  //DYB Write  -24bit or 32bit address
#endif
#ifdef INCL_SPRPCmd
#define SPI_SPRP_CMD                (0xFB) // Set Pointer Region Protection
#endif
#ifdef INCL_PPBRD_Cmd
#define SPI_PPBRD_CMD               (0xFC)  //PPB Read   -24bit or 32bit address
#endif
#ifdef INCL_PPBP_Cmd
#define SPI_PPBP_CMD                (0xFD)   //PPB Write -24bit or 32bit address
#endif
#ifdef INCL_SPI_READMODERESETCMD
#define SPI_READMODE_RESET_CMD      (0xFF)  // Continuous Read Mode Reset
#endif
#ifdef INCL_SPI_MODEBITRESET
#define SPI_MBR_CMD                 (0xFF) // Mode Bit Reset
#endif
#ifdef INCL_RCVRCmd
#define SPI_RCVR_CMD                (0x00) // Reserved: Initiate recovery mode (manually refreshing ECC)
#endif
#ifdef INCL_RCSPCmd
#define SPI_RCSP_CMD                (0x00) // Reserved: Recovery suspend
#endif
#ifdef INCL_RCRSCmd
#define SPI_RCRS_CMD                (0x00) // Reserved: Recovery resume
#endif

/* Voltage States */
#define SLLD_P_VIL                  (0x0001)
#define SLLD_P_VIH                  (0x0002)

/* LLD System Specific Typedefs */
typedef unsigned short LLD_UINT16;  /* 16 bits wide */
typedef unsigned long  LLD_UINT32;  /* 32 bits wide */
typedef LLD_UINT32     ADDRESS;     /* Used for system level addressing */

/* SLLD System Specific Typedefs */
typedef unsigned char  BYTE;   /* 8 bits wide */
typedef unsigned short WORD;   /* 16 bits wide */
typedef unsigned long  DWORD;  /* 32 bits wide */
typedef BYTE FLASHDATA;

/* SLLD Internal Data Types */
//typedef DWORD PARAM;            /* MUST be at least 32 bits wide */
typedef unsigned long BYTECOUNT;  /* used for multi-byte operations */

/* boolean macros */
#ifndef TRUE
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

typedef enum
{
    VOLATILE_WR = 0,
    NON_VOLATILE_WR
}WRR_mode;

/* data mask */
#define B0_MASK                  (0x01)
#define B1_MASK                  (0x02)
#define B2_MASK                  (0x04)
#define B3_MASK                  (0x08)
#define B5_MASK                  (0x20)
#define B6_MASK                  (0x40)
#define B7_MASK                  (0x80)

#define LLD_DEV_READ_MASK        (0xFF)
#define LLD_BYTES_PER_OP         (0x01)

/*RDAR read any register address definition*/
// Non-Volatile Status and Configuration Registers
#define    SR1NV         0x00000000
#define    CR1NV         0x00000002
#define    CR2NV         0x00000003
#define    CR3NV         0x00000004
#define    CR4NV         0x00000005
#define    NVDLP         0x00000005
// Non-Volatile Data Learning Register
#define    NVDLR         0x00000010
#define    PASS7_0       0x00000020
//Non-Volatile Password Register
#define    PASS15_8      0x00000021
#define    PASS23_160    0x00000022
#define    PASS31_24     0x00000023
#define    PASS39_32     0x00000024
#define    PASS47_40     0x00000025
#define    PASS55_48     0x00000026
#define    PASS63_56     0x00000027
//#ifdef FL_L
#define    IRP7_0           0x00000030
#define    IRP15_8         0x00000031
//#else
#define    ASPR7_0       0x00000030
#define    ASPR15_8      0x00000031
//#endif
#define    PRPRA15_A8       0x00000039
#define    PRPRA23_A16     0x0000003A
#define    PRPRA31_A24     0x0000003B
#define    SR1V          0x00800000
//Volatile Status and Configuration Registers
#define    SR2V          0x00800001
#define    CR1V          0x00800002
#define    CR2V          0x00800003
#define    CR3V          0x00800004
#define    CR4V          0x00800005
#define    VDLP          0x00800005
// Volatile Data Learning Register
#define    VDLR          0x00800010
//Volatile PPB Lock Register
#define    PPBL          0x00800040
#define    PR              0x00800040

/*Device status */
/*Flash Software protect status */
typedef enum {
    FLASH_SOFTWARE_UNPROTECTED = 0,
    FLASH_SOFTWARE_PROTECTED
} DEV_SOFTWARE_PROTECT_STATUS;

// Flash embedded operation status
typedef enum {
    dev_status_unknown = 0,
    dev_not_busy,
    dev_program_error,
    dev_erase_error,
    dev_suspend,
    dev_busy
} DEVSTATUS;

// SLLD Returned values
typedef enum {
    SLLD_OK = 0x0,
    SLLD_E_DEVICE_SOFTWARE_PROTECTED,
    SLLD_E_HAL_ERROR = 0x200,
    SLLD_ERROR = 0xFFFF
} SLLD_STATUS;

// FLASH_RD and FLASH_WR functions needed defines
#define ADDRESS_NOT_USED 0xFFFFFFFF
#define BUFFER_NOT_USED  (BYTE*)0

extern BYTE modebit_char;

// Include Nor Super Tests
//#define NST_TESTS

/* Include for Competitive Analysis Test */
/* #define CA_TEST */

/* public function prototypes */

/* Operation Functions */

#ifdef INCL_ReadSecurityCmd
extern SLLD_STATUS slld_ReadSecurityCmd 
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                           // device address given by system
        BYTE     *read_buf,                           // data buffer
        BYTECOUNT len_in_bytes                        // number of bytes
);
#endif

#ifdef INCL_ProgramSecurityCmd
extern SLLD_STATUS slld_ProgramSecurityCmd 
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                           // device address given by system
        BYTE     *program_buf,                        // variable in which to store programmed data
        BYTECOUNT len_in_bytes                        // number of bytes
);

extern SLLD_STATUS slld_ProgramSecurityOp
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,        // device address given by system
        BYTE                    *program_buf,       // variable in which to store programmed data
        BYTECOUNT          len_in_bytes,            // number of bytes
        DEVSTATUS          *dev_status_ptr          // variable to store device status
);
#endif

#ifdef INCL_EraseSecurityCmd
extern SLLD_STATUS slld_EraseSecurityCmd 
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr                           // device address given by system
);

extern SLLD_STATUS slld_EraseSecurityOp
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,        // device address given by system
        DEVSTATUS          *dev_status_ptr          // variable to store device status
);
#endif

#ifdef INCL_ReadIdDualCmd
extern SLLD_STATUS slld_ReadIdDualCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,       // device address given by system
        BYTE                    *read_buf,         // variable in which to store read data
        BYTE                    modebit,           // mode bit
        BYTECOUNT          len_in_bytes            // length in bytes
);
#endif

#ifdef INCL_ReadIdQuadCmd
extern SLLD_STATUS slld_ReadIdQuadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,      // device address given by system
        BYTE                    *read_buf,        // variable in which to store read data
        BYTE                    modebit,          // mode bit
        BYTECOUNT          len_in_bytes           // length in bytes
);
#endif

#ifdef INCL_BE32KBCmd
extern SLLD_STATUS slld_BE32KBCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                         // device address given by system
);

extern SLLD_STATUS slld_BE32KBOp
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,                         // device address given by system
        DEVSTATUS          *dev_status_ptr         // variable to store device status
);
#endif

#ifdef INCL_WordReadQuadCmd
extern SLLD_STATUS slld_WordReadQuadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,       // device address given by system
        BYTE                    *read_buf,         // variable in which to store read data
        BYTE                    modebit,           // mode bit
        BYTECOUNT          len_in_bytes            // length in bytes
);
#endif

#ifdef INCL_ReadUniqueIDCmd
extern SLLD_STATUS slld_ReadUniqueIDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *read_buf                        // variable in which to store read data
);
#endif

#ifdef INCL_OctalWordReadQuadCmd
extern SLLD_STATUS slld_OctalWordReadQuadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,       // device address given by system
        BYTE                    *read_buf,         // variable in which to store read data
        BYTE                    modebit,           // mode bit
        BYTECOUNT          len_in_bytes            // length in bytes
);
#endif

#ifdef INCL_WriteVolatileCmd
extern SLLD_STATUS slld_WriteVolatileCmd(BYTE device_num); /* device number */
#endif

#ifdef INCL_Read_IDCmd
extern SLLD_STATUS slld_Read_IDCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                       /* variable in which to store read data */
);
#endif

#ifdef INCL_RDIDCmd
extern SLLD_STATUS slld_RDIDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *target,                    /* variable in which to store read data */
        BYTECOUNT  len_in_bytes               /* number of bytes to read */
);
#endif

#ifdef INCL_READ_IDENTIFICATIONCmd
extern SLLD_STATUS slld_Read_IdentificationCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target,                      /* variable in which to store read data */
        ADDRESS  addr                         /* address offset for the command */
);
#endif

/* Registers Operations */

#ifdef INCL_RDSRCmd
extern SLLD_STATUS slld_RDSRCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                       /* variable in which to store read data */
);
#endif

#ifdef INCL_RASPCmd
extern SLLD_STATUS slld_RASPCmd
(
        BYTE  device_num,            /* device number */
        WORD    *target                       /* variable in which to store read data */
);
#endif

#ifdef INCL_BRRDCmd
extern SLLD_STATUS slld_BRRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                       /* variable in which to store the bank addressing register value */
);
#endif

#ifdef INCL_ABRDCmd
extern SLLD_STATUS slld_ABRDCmd
(
        BYTE  device_num,            /* device number */
        DWORD    *target                       /* variable in which to store the autoboot register value */
);
#endif

#ifdef INCL_ECCRDCmd
extern SLLD_STATUS slld_ECCRDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,                    /* cache line address given by system */
        BYTE     *target                      /* variable in which to store read data */
);
#endif

#ifdef INCL_RPWDCmd
extern SLLD_STATUS slld_RPWDCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                       /* variable in which to store read data */
);
#endif

#ifdef INCL_WRENCmd
extern SLLD_STATUS slld_WRENCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_WRDICmd
extern SLLD_STATUS slld_WRDICmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_WRSRCmd
extern SLLD_STATUS slld_WRSRCmd
(
        BYTE  device_num,            /* device number */
        BYTE   *data_buf                      /* variable containing data to program */
);

extern SLLD_STATUS slld_WRSROp
(
        BYTE  device_num,            /* device number */
        BYTE       *data_buf,                 /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

extern void slld_VersionCmd (void); /* device number */

#ifdef INCL_SRSTCmd
extern SLLD_STATUS slld_SRSTCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_PPB_PGCmd
extern SLLD_STATUS slld_PPB_PGCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_PPB_PGOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_DYB_PGCmd
extern SLLD_STATUS slld_DYB_PGCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                   /* device address given by system */
        BYTE     *data_buf                    /* variable containing data to program */
);

extern SLLD_STATUS slld_DYB_PGOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_ERS_SSPCmd
extern SLLD_STATUS slld_ERS_SSPCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_ERS_RESCmd
extern SLLD_STATUS slld_ERS_RESCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_RCSPCmd
extern SLLD_STATUS slld_RCSPCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_RCRSCmd
extern SLLD_STATUS slld_RCRSCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_RCVRCmd
extern SLLD_STATUS slld_RCVRCmd (BYTE device_num); /* device number */
#endif

/* Power Saving Mode Operation */
#ifdef INCL_DPCmd
extern SLLD_STATUS slld_DPCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_BRACCCmd
extern SLLD_STATUS slld_BRACCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_SPCmd
extern SLLD_STATUS slld_SPCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_RESCmd
extern SLLD_STATUS slld_RESCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_CLSRCmd
extern SLLD_STATUS slld_ClearStatusRegisterCmd (BYTE device_num); /* device number */
#endif

#ifdef INCL_WRRCmd
SLLD_STATUS slld_WRRCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *status_val,                  /* variable containing data to program the status register */
        BYTE    *config_val,                   /* variable containing data to program the config register */
        BYTE    *status2_val                 /* variable containing data to program the status register2 */
);

SLLD_STATUS slld_WRROp
(
        BYTE  device_num,            /* device number */
        BYTE       *status_val,               /* variable containing data to program the status register */
        BYTE       *config_val,               /* variable containing data to program the config register */
        BYTE       *status2_val,              /* variable containing data to program the status register2 */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_WASPCmd
SLLD_STATUS slld_WASPCmd
(
        BYTE  device_num,            /* device number */
        WORD    *asp_val                      /* variable containing data to program to the ASP register */
);

SLLD_STATUS slld_WASPOp
(
        BYTE  device_num,            /* device number */
        WORD       *asp_val,                  /* variable containing data to program to the ASP register */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_BRWRCmd
SLLD_STATUS slld_BRWRCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *bnk_val                      /* variable containing data to program to the bank addressing register */
);
#endif

#ifdef INCL_ABWRCmd
SLLD_STATUS slld_ABWRCmd
(
        BYTE  device_num,            /* device number */
        DWORD    *abt_val                      /* variable containing data to program to the autoboot register */
);

SLLD_STATUS slld_ABWROp
(
        BYTE  device_num,            /* device number */
        DWORD       *abt_val,                  /* variable containing data to program to the autoboot register */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_WPWDCmd
SLLD_STATUS slld_WPWDCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *target                      /* variable containing data to program to the ASP password */
);

SLLD_STATUS slld_WPWDOp
(
        BYTE  device_num,            /* device number */
        BYTE       *target,                   /* variable containing data to program to the ASP password */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_RCRCmd
SLLD_STATUS slld_RCRCmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                       /* variable in which to store read data */
);
#endif

#ifdef INCL_OTPRCmd
SLLD_STATUS slld_OTPRCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *target,                    /* variable in which to store read data */
        BYTECOUNT  len_in_bytes               /* number of bytes to read */
);
#endif

#ifdef INCL_OTPPCmd
SLLD_STATUS slld_OTPPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr,                    /* device address given by system */
        BYTE    *data_buf                     /* variable containing data to program */
);

SLLD_STATUS slld_OTPPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

/* Command Functions */

/* Read Operations */

#ifdef INCL_ReadCmd
extern SLLD_STATUS slld_ReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *target,                   /* variable in which to store read data */
        BYTECOUNT   len_in_bytes              /* number of bytes on which to operate */
);
#endif

#ifdef INCL_Read_4BCmd
extern SLLD_STATUS slld_Read_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *target,                   /* variable in which to store read data */
        BYTECOUNT   len_in_bytes              /* number of bytes on which to operate */
);
#endif

#ifdef INCL_Fast_ReadCmd
extern SLLD_STATUS slld_Fast_ReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *target,                   /* variable in which to store read data */
        BYTECOUNT   len_in_bytes              /* number of bytes on which to operate */
);
#endif

#ifdef INCL_Fast_Read_4BCmd
extern SLLD_STATUS slld_Fast_Read_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *target,                   /* variable in which to store read data */
        BYTECOUNT   len_in_bytes              /* number of bytes on which to operate */
);
#endif

#ifdef INCL_DUALOUTPUT_READCmd
extern SLLD_STATUS slld_DualIOReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *target,                    /* variable in which to store read data */
        BYTECOUNT  len_in_bytes               /* number of bytes to read */
);
#endif

#ifdef INCL_DUALOUTPUT_READ_4BCmd
extern SLLD_STATUS slld_DualIORead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *target,                    /* variable in which to store read data */
        BYTECOUNT  len_in_bytes               /* number of bytes to read */
);
#endif

#ifdef INCL_DORCmd
extern SLLD_STATUS slld_DORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_DORCmd */

#ifdef INCL_4DORCmd
extern SLLD_STATUS slld_4DORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_4DORCmd */

#ifdef INCL_QUADOUTPUT_READCmd
extern SLLD_STATUS slld_QuadIOReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *target,                    /* variable in which to store read data */
        BYTECOUNT  len_in_bytes               /* number of bytes to read */
);
#endif

#ifdef INCL_QUADOUTPUT_READ_4BCmd
extern SLLD_STATUS slld_QuadIORead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *target,                    /* variable in which to store read data */
        BYTECOUNT  len_in_bytes               /* number of bytes to read */
);
#endif

#ifdef INCL_QORCmd
extern SLLD_STATUS slld_QORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_QORCmd */

#ifdef INCL_4QORCmd
extern SLLD_STATUS slld_4QORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_4QORCmd */

#ifdef INCL_DUALOUTPUT_HP_READCmd
extern SLLD_STATUS slld_DualIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,     // device address given by system */
        BYTE                    *target,         // variable in which to store read data */
        BYTE                    modebit,         // The read mode to be passed to the device */
        BYTECOUNT          len_in_bytes          // number of bytes to read */
);
#endif

#ifdef INCL_DUALOUTPUT_HP_READ_4BCmd
extern SLLD_STATUS slld_DualIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,       // device address given by system
        BYTE                    *target,           // variable in which to store read data
        BYTE                    modebit,           // The read mode to be passed to the device
        BYTECOUNT          len_in_bytes            // number of bytes to read
);
#endif

#ifdef INCL_DIORCmd
extern SLLD_STATUS slld_DIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE      modebit,        /* mode bit */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_DIORCmd */

#ifdef INCL_4DIORCmd
extern SLLD_STATUS slld_4DIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE      modebit,        /* mode bit */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_4DIORCmd */

#ifdef INCL_QUADOUTPUT_HP_READCmd
extern SLLD_STATUS slld_QuadIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,        // device address given by system
        BYTE                    *target,            // variable in which to store read data
        BYTE                    modebit,            // The read mode to be passed to the device
        BYTECOUNT          len_in_bytes             // number of bytes to read
);
#endif

#ifdef INCL_QUADOUTPUT_HP_READ_4BCmd
extern SLLD_STATUS slld_QuadIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                    sys_addr,         // device address given by system
        BYTE                    *target,             // variable in which to store read data
        BYTE                    modebit,             // The read mode to be passed to the device
        BYTECOUNT          len_in_bytes              // number of bytes to read
);
#endif

#ifdef INCL_QIORCmd
extern SLLD_STATUS slld_QIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE      modebit,        /* mode bit */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_QIORCmd */

#ifdef INCL_4QIORCmd
extern SLLD_STATUS slld_4QIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,        /* device address given by system */
        BYTE      *target,          /* variable in which to store read data */
        BYTE      modebit,        /* mode bit */
        BYTECOUNT  len_in_bytes     /* number of bytes to read */
);
#endif /* INCL_4QIORCmd */

#ifdef INCL_DDR_Fast_ReadCmd
extern SLLD_STATUS slld_DDR_Fast_ReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,     /* device address given by system*/
        BYTE                *target,         /* variable in which to store read data*/
        BYTE                modebit,         /* The read mode to be passed to the device*/
        BYTECOUNT        len_in_bytes        /* number of bytes to read*/
);
#endif

#ifdef INCL_DDR_Fast_4BReadCmd
extern SLLD_STATUS slld_DDR_Fast_4BReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      /* device address given by system*/
        BYTE                *target,          /* variable in which to store read data*/
        BYTE                modebit,          /* The read mode to be passed to the device*/
        BYTECOUNT        len_in_bytes         /* number of bytes to read*/
);
#endif

#ifdef INCL_DDR_DUALOUTPUT_HP_READCmd
extern SLLD_STATUS slld_DDR_DualIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      /* device address given by system*/
        BYTE                *target,          /* variable in which to store read data*/
        BYTE                modebit,          /* mode bit*/
        BYTECOUNT        len_in_bytes         /* number of bytes to read*/
);
#endif

#ifdef INCL_DDR_DUALOUTPUT_HP_READ_4BCmd
extern SLLD_STATUS slld_DDR_DualIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      /* device address given by system*/
        BYTE                *target,          /* variable in which to store read data*/
        BYTE                modebit,          /* The read mode to be passed to the device*/
        BYTECOUNT        len_in_bytes         /* number of bytes to read*/
);
#endif

#ifdef INCL_DDR_QUADOUTPUT_HP_READCmd
extern SLLD_STATUS slld_DDR_QuadIOHPReadCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      /* device address given by system*/
        BYTE                *target,          /* variable in which to store read data*/
        BYTE                modebit,          /* mode bit*/
        BYTECOUNT        len_in_bytes         /* number of bytes to read*/
);
#endif

#ifdef INCL_DDR_QUADOUTPUT_HP_READ_4BCmd
extern SLLD_STATUS slld_DDR_QuadIOHPRead_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      /* device address given by system*/
        BYTE                *target,          /* variable in which to store read data*/
        BYTE                modebit,          /* The read mode to be passed to the device*/
        BYTECOUNT        len_in_bytes         /* number of bytes to read*/
);
#endif

#ifdef INCL_DDRQIORCmd
extern SLLD_STATUS slld_DDRQIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
);
#endif /* INCL_DDRQIORCmd */

#ifdef INCL_4DDRQIORCmd
extern SLLD_STATUS slld_4DDRQIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
);
#endif /* INCL_4DDRQIORCmd */

/* Erase Operations */

#ifdef INCL_BulkEraseCmd
extern SLLD_STATUS slld_BECmd (BYTE device_num); /* device number */

extern SLLD_STATUS slld_BEOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);

extern SLLD_STATUS slld_BE1Cmd (BYTE device_num); /* device number */

extern SLLD_STATUS slld_BE1Op
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_SECmd
extern SLLD_STATUS slld_SECmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_SEOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_SE_4BCmd
extern SLLD_STATUS slld_SE_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_SE_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_HBECmd
extern SLLD_STATUS slld_HBECmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_HBEOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif /* INCL_HBEOp */

#ifdef INCL_4HBECmd
extern SLLD_STATUS slld_4HBECmd
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr               /* device address given by system */
);
#endif /* INCL_4HBEOp */

#ifdef INCL_4BECmd
extern SLLD_STATUS slld_4BECmd 
(
        BYTE      device_num,                     //device number
        ADDRESS  sys_addr
);
#endif  /* INCL_4BECmd */

#ifdef INCL_CECmd
extern SLLD_STATUS slld_CECmd 
(
        BYTE      device_num                     //device number
);

extern SLLD_STATUS slld_CEOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif /* INCL_CECmd */

#ifdef INCL_CE1Cmd
extern SLLD_STATUS slld_CE1Cmd 
(
        BYTE      device_num                     //device number
);

extern SLLD_STATUS slld_CE1Op
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_CE1Cmd */

#ifdef INCL_BLOCKERASECmd
extern SLLD_STATUS slld_BlockEraseCmd 
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr
);

extern SLLD_STATUS slld_BlockEraseOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,             /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif

/* Program Operations */

#ifdef INCL_PPCmd
extern SLLD_STATUS slld_PPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS      sys_addr,                /* device address given by system */
        BYTE        *data_buf,                /* variable containing data to program */
        BYTECOUNT    len_in_bytes             /* number of bytes on which to operate */
);

extern SLLD_STATUS slld_PPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_PP_4BCmd
extern SLLD_STATUS slld_PP_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS      sys_addr,                /* device address given by system */
        BYTE        *data_buf,                /* variable containing data to program */
        BYTECOUNT    len_in_bytes             /* number of bytes on which to operate */
);

extern SLLD_STATUS slld_PP_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_P4ECmd
extern SLLD_STATUS slld_P4ECmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_P4EOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_P4E_4BCmd
extern SLLD_STATUS slld_P4E_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                          /* device address given by system */
);

extern SLLD_STATUS slld_P4E_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_P8ECmd
extern SLLD_STATUS slld_P8ECmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_P8EOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_P8E_4BCmd
extern SLLD_STATUS slld_P8E_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                     /* device address given by system */
);

extern SLLD_STATUS slld_P8E_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif


#ifdef INCL_QPPCmd
extern SLLD_STATUS slld_QPPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes on which to operate */
);

extern SLLD_STATUS slld_QPPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_QPP_4BCmd
extern SLLD_STATUS slld_QPP_4BCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes on which to operate */
);

extern SLLD_STATUS slld_QPP_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE       *data_buf,                  /* variable containing data to program */
        BYTECOUNT   len_in_bytes,              /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr             /* variable to store device status */
);
#endif

#ifdef INCL_QPP2md
extern SLLD_STATUS slld_QPP2Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
);
#endif

#ifdef INCL_MPMCmd
extern SLLD_STATUS slld_MPMCmd (BYTE device_num); /* device number */
#endif

extern SLLD_STATUS slld_Poll
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);

extern SLLD_STATUS slld_StatusGet
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);

#ifdef INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK
extern SLLD_STATUS slld_SoftwareProtectStatusGet
(
        DEV_SOFTWARE_PROTECT_STATUS*  dev_softwareprotect_status_ptr
        /* variable to store device software protect status */
);
#endif // INCLUDE_SOFTWARE_PROTECT_STATUS_CHECK

#ifdef INCL_BufferedProgramOp
extern SLLD_STATUS slld_BufferedProgramOp
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,                  /* device address given by system */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes,              /* number of bytes on which to operate */
        DEVSTATUS *dev_status_ptr             /* variable to store device status */
);
#endif

#ifdef INCL_BufferedProgram_4BOp
extern SLLD_STATUS slld_BufferedProgram_4BOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_BlockProtectOp
extern SLLD_STATUS slld_BlockProtectOp
(
        BYTE  device_num,            /* device number */
        BYTE        bpb_value,                /* block protect bit value */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_QuadWriteOp
extern SLLD_STATUS slld_QuadWriteOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes on which to operate */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
#endif

#ifdef INCL_RDSR2Cmd
extern SLLD_STATUS slld_RDSR2Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                        /* variable in which to store read data */
);
#endif

#ifdef INCL_DLPRDCmd
extern SLLD_STATUS slld_DLPRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
);
#endif

#ifdef INCL_PNVDLRCmd
extern SLLD_STATUS slld_PNVDLRCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
);
#endif

#ifdef INCL_WVDLRCmd
extern SLLD_STATUS slld_WVDLRCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
);
#endif

#ifdef INCL_PGSPCmd
extern SLLD_STATUS slld_PGSPCmd(BYTE device_num); /* device number */
#endif

#ifdef INCL_PGRSCmd
extern SLLD_STATUS slld_PGRSCmd(BYTE device_num); /* device number */
#endif

#ifdef INCL_PLBWRCmd
extern SLLD_STATUS slld_PLBWRCmd(BYTE device_num); /* device number */
#endif

#ifdef INCL_PLBRDCmd
extern SLLD_STATUS slld_PLBRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf,                  /* variable containing data to program */
        BYTECOUNT  len_in_bytes               /* number of bytes to operate */
);
#endif

#ifdef INCL_DYB_RDCmd
extern SLLD_STATUS slld_DYB_RDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* data buffer to write */
);
#endif

#ifdef INCL_PPB_RDCmd
extern SLLD_STATUS slld_PPB_RDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                      /* device address given by system */
        BYTE     *data_buf                       /* variable containing data to program */
);
#endif /* INCL_PPB_RDCmd */

#ifdef INCL_PPB_ERSCmd
extern SLLD_STATUS slld_PPB_ERSCmd(BYTE device_num); /* device number */
#endif /* INCL_PPB_ERSCmd */

#ifdef INCL_PASSRDCmd
SLLD_STATUS slld_PASSRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *data_buf                       /* variable containing data to program */
);
#endif /* INCL_PASSRDCmd */

#ifdef INCL_PASSPCmd
SLLD_STATUS slld_PASSPCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *data_buf                       /* variable containing data to program */
);
#endif /* INCL_PASSPCmd */

#ifdef INCL_PASSUCmd
SLLD_STATUS slld_PASSUCmd
(
        BYTE  device_num,            /* device number */
        BYTE     *data_buf                       /* variable containing data to program */
);
#endif /* INCL_PASSUCmd */

#ifdef INCL_RDARCmd
extern SLLD_STATUS slld_RDARCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS             reg_addr,         /* register address given by device*/
        BYTE                *target          /* variable in which to store read data*/
);
#endif //INCL_RDARCmd

#ifdef INCL_WRARCmd
extern SLLD_STATUS slld_WRARCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS             reg_addr,         /* register address given by device*/
        BYTE                *data_buf         /*variable containing data to program*/
);

extern SLLD_STATUS slld_WRAR_Op
(
        BYTE  device_num,            /* device number */
        ADDRESS             reg_addr,         /*register address given by device*/
        BYTE                *data_buf,        /*variable containing data to program*/
        DEVSTATUS           *dev_status_ptr   /* variable to store device status */
);
#endif

#ifdef INCL_RSTENCmd
extern SLLD_STATUS slld_RSTENCmd(BYTE device_num); /* device number */
#endif //INCL_RSTENCmd

#ifdef INCL_RSTCmd
extern SLLD_STATUS slld_RSTCmd(BYTE device_num); /* device number */
#endif //INCL_RSTCmd

#ifdef INCL_RDQIDCmd
extern SLLD_STATUS slld_RDQIDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *target,                   /*variable in which to store read data*/
        BYTECOUNT  len_in_bytes              /* number of bytes to read*/
);
#endif /* INCL_RDQIDCmd */

#ifdef INCL_EESCmd
extern SLLD_STATUS slld_ESSCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr      /*device address given by system*/
);
#endif //INCL_ESSCmd

#ifdef INCL_DYBRD_Cmd
extern SLLD_STATUS slld_DYBRD_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                   /* device address given by system */
        BYTE     *data_buf                    /* data buffer to write */
);
#endif /* INCL_DYBRD_Cmd */

#ifdef INCL_PPBRD_Cmd
extern SLLD_STATUS slld_PPBRD_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                  /*device address given by system*/
        BYTE     *data_buf                   /*variable containing data to program*/
);

#endif /* INCL_PPBRD_Cmd */

#ifdef INCL_PPBP_Cmd
extern SLLD_STATUS slld_PPBP_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS  sys_addr                   /*device address given by system*/
);

extern SLLD_STATUS slld_PPBP_Op
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,               /*device address given by system*/
        DEVSTATUS  *dev_status_ptr          /* variable to store device status*/
);
#endif /* INCL_PPBP_Op */

#ifdef INCL_DYBWR_Cmd
extern SLLD_STATUS slld_DYBWR_Cmd
(
        BYTE  device_num,            /* device number */
        ADDRESS   sys_addr,                 /*device address given by system*/
        BYTE     *data_buf                  /*variable containing data to program*/
);

extern SLLD_STATUS slld_DYBWR_Op
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,               /*device address given by system*/
        BYTE       *data_buf,               /*variable containing data to program*/
        DEVSTATUS  *dev_status_ptr          /*variable to store device status*/
);
#endif /* INCL_DYBWR_Op */

#ifdef INCL_EPR_Cmd
extern SLLD_STATUS slld_EPR_Cmd (BYTE device_num); /* device number */
#endif /* INCL_EPR_Cmd */

#ifdef INCL_EPS_Cmd
extern SLLD_STATUS slld_EPS_Cmd (BYTE device_num); /* device number */
#endif /* INCL_EPS_Cmd */

#ifdef INCL_4BAM_Cmd
extern SLLD_STATUS slld_4BAM_Cmd (BYTE device_num); /* device number */
#endif /* INCL_4BAM_Cmd */

#ifdef INCL_SPI_READSFDPCMD
extern SLLD_STATUS slld_ReadSFDPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS    sys_addr,             // device address given by system
        BYTE      *read_buf               // data buffer
);
#endif

#ifdef INCL_SPI_READMODERESETCMD
extern SLLD_STATUS slld_ReadModeResetCmd(BYTE  device_num);  /* device number */
#endif

#ifdef INCL_SPI_BURSTWRAPCMD
extern SLLD_STATUS slld_SetBurstWrapCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *wrapbit_buf
);
#endif

#ifdef INCL_RDSR3Cmd
extern SLLD_STATUS slld_RDSR3Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                        /* variable in which to store read data */
);
#endif

#ifdef INCL_WRSRegCmd
extern SLLD_STATUS slld_WRSRegCmd
(
        BYTE  device_num,                     /* device number */
        BYTE        *status_val,         /* variable array containing data to program the status register */
        BYTECOUNT   number               /* number of register to program ,1~3 Status Register0~3*/
);

extern SLLD_STATUS slld_WRSRegOp
(
        BYTE  device_num,                     /* device number */
        BYTE        *status_val,          /* variable containing data to program the status register */
        WRR_mode    mode,                 /*WRR_mode ,VOLATILE_WR NON_VOLATILE_WR*/
        BYTECOUNT   number,               /* number of register to program ,1~3 Status Register0~3*/
        DEVSTATUS   *dev_status_ptr       /* variable to store device status */
);
#endif

#ifdef INCL_RDCR2Cmd
extern SLLD_STATUS slld_RDCR2Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                   /* variable in which to store read data */
);
#endif /* INCL_RDCR2Cmd */

#ifdef INCL_RDCR3Cmd
extern SLLD_STATUS slld_RDCR3Cmd
(
        BYTE  device_num,            /* device number */
        BYTE    *target                   /* variable in which to store read data */
);
#endif /* INCL_RDCR3Cmd */

#ifdef INCL_IRPRDCmd
extern SLLD_STATUS slld_IRPRDCmd
(
        BYTE      device_num,                     //device number
        WORD    *target                        /* variable in which to store read data */
);
#endif /* INCL_IRPRDCmd */

#ifdef INCL_IRPPCmd
extern SLLD_STATUS slld_IRPPCmd
(
        BYTE  device_num,            /* device number */
        WORD    *irp_val                 /* variable containing data to program to the ASP register */
);

extern SLLD_STATUS slld_IRPPOp
(
        BYTE  device_num,            /* device number */
        WORD       *irp_val,             /* variable containing data to program to the ASP register */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_IRPPOp */

#ifdef INCL_QPIENCmd
extern SLLD_STATUS slld_QPIENCmd
(
        BYTE  device_num            /* device number */
);
#endif /* INCL_QPIENCmd */

#ifdef INCL_QPIEXCmd
extern SLLD_STATUS slld_QPIEXCmd
(
        BYTE  device_num            /* device number */
);
#endif /* INCL_QPIEXCmd */

#ifdef INCL_4BENCmd
extern SLLD_STATUS slld_4BENCmd 
(
        BYTE device_num              /* device number */
);
#endif /* INCL_4BEN_Cmd */

#ifdef INCL_4BEXCmd
extern SLLD_STATUS slld_4BEXCmd 
(
        BYTE device_num              /* device number */
);
#endif /* INCL_4BEX_Cmd */

#ifdef INCL_SECRRCmd 
extern SLLD_STATUS slld_SECRRCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                      // device address given by system
        BYTE     *read_buf,                      // data buffer
        BYTECOUNT len_in_bytes                   // number of bytes
);
#endif /* INCL_SECRRCmd */

#ifdef INCL_SECRPCmd 
extern SLLD_STATUS slld_SECRPCmd 
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr,                 // device address given by system
        BYTE     *program_buf,              // data buffer
        BYTECOUNT len_in_bytes              // number of bytes
);

extern SLLD_STATUS slld_SECRPOp
(
        BYTE      device_num,                     //device number
        ADDRESS          sys_addr,                           // device address given by system
        BYTE             *program_buf,                       // data buffer
        BYTECOUNT        len_in_bytes,                       // number of bytes
        DEVSTATUS        *dev_status_ptr                     // variable to store device status
);
#endif /* INCL_SECRPCmd */

#ifdef INCL_SECRECmd 
extern SLLD_STATUS slld_SECRECmd
(
        BYTE      device_num,                     //device number
        ADDRESS   sys_addr                  // device address given by system
);

extern SLLD_STATUS slld_SECREOp
(
        BYTE            device_num,                     //device number
        ADDRESS     sys_addr,                 // device address given by system
        DEVSTATUS  *dev_status_ptr            // variable to store device status
);
#endif /* INCL_SECRECmd */

#ifdef INCL_RUIDCmd
extern SLLD_STATUS slld_RUIDCmd
(
        BYTE      device_num,                     //device number
        BYTE      *read_buf        // data buffer
);
#endif /* INCL_RUIDCmd */

#ifdef INCL_WRENVCmd
extern SLLD_STATUS slld_WRENVCmd
(
        BYTE device_num              /* device number */
);
#endif /* INCL_WRENVCmd */

#ifdef INCL_IBLRDCmd
extern SLLD_STATUS slld_IBLRDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE                *target           /*variable in which to store read data*/
);
#endif /* INCL_IBLRDCmd */

#ifdef INCL_4IBLRDCmd
extern SLLD_STATUS slld_4IBLRDCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE                *target           /*variable in which to store read data*/
);
#endif /* INCL_4IBLRDCmd */

#ifdef INCL_IBLCmd
extern SLLD_STATUS slld_IBLCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr                 /* device address given by system */
);
#endif /* INCL_IBLCmd */

#ifdef INCL_4IBLCmd
extern SLLD_STATUS slld_4IBLCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr                 /* device address given by system */
);
#endif /* INCL_4IBLCmd */

#ifdef INCL_IBLOp
extern SLLD_STATUS slld_IBLOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_IBLOp */

#ifdef INCL_IBULCmd
extern SLLD_STATUS slld_IBULCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr                 /* device address given by system */
);
#endif /* INCL_IBULCmd */

#ifdef INCL_4IBULCmd
extern SLLD_STATUS slld_4IBULCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr                 /* device address given by system */
);
#endif /* INCL_4IBULCmd */

#ifdef INCL_IBULOp
extern SLLD_STATUS slld_IBULOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_IBULOp */

#ifdef INCL_GBLCmd
extern SLLD_STATUS slld_GBLCmd
(
        BYTE  device_num            /* device number */
);

extern SLLD_STATUS slld_GBLOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_GBLOp */

#ifdef INCL_GBULCmd
extern SLLD_STATUS slld_GBULCmd
(
        BYTE  device_num            /* device number */
);

extern SLLD_STATUS slld_GBULOp
(
        BYTE  device_num,            /* device number */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_GBULOp */

#ifdef INCL_SPRPCmd
extern SLLD_STATUS slld_SPRPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr                 /* device address given by system */
);
#endif /* INCL_SPRPCmd */

#ifdef INCL_4SPRPCmd
extern SLLD_STATUS slld_4SPRPCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr                 /* device address given by system */
);
#endif /* INCL_4SPRPCmd */

#ifdef INCL_SPRPOp
extern SLLD_STATUS slld_SPRPOp
(
        BYTE  device_num,            /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        DEVSTATUS  *dev_status_ptr       /* variable to store device status */
);
#endif /* INCL_SPRPOp */

#ifdef INCL_PRLCmd
extern SLLD_STATUS slld_PRLCmd(BYTE device_num);
#endif /* INCL_PRLCmd */

#ifdef INCL_PRRDCmd
extern SLLD_STATUS slld_PRRDCmd
(
        BYTE  device_num,            /* device number */
        BYTE      *data_buf                  /* variable containing data to program */
);
#endif /* INCL_PRRDCmd */

#ifdef INCL_SPI_SETBUSRTLENGTH
extern SLLD_STATUS slld_SBLCmd
(
        BYTE  device_num,            /* device number */
        BYTE  *wrapbit_buf                     // variable in which to store wrapbit data
);
#endif

#ifdef  INCL_SPI_MODEBITRESET
extern SLLD_STATUS slld_MBRCmd (BYTE device_num);
#endif /* INCL_SPI_MODEBITRESET */

#ifdef INCL_DDRDIORCmd
SLLD_STATUS slld_DDRDIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // mode bit
        BYTECOUNT        len_in_bytes         // number of bytes to read
);
#endif /* INCL_DDRDIORCmd */

#ifdef INCL_4DDRDIORCmd
SLLD_STATUS slld_4DDRDIORCmd
(
        BYTE  device_num,            /* device number */
        ADDRESS                sys_addr,      // device address given by system
        BYTE                *target,          // variable in which to store read data
        BYTE                modebit,          // The read mode to be passed to the device
        BYTECOUNT        len_in_bytes         // number of bytes to read
);
#endif /* INCL_4DDRDIORCmd */

/*****************************************************
 * Define Flash read/write macro to be used by SLLD   *
 *****************************************************/
#ifdef TRACE
#define FLASH_WR(dn, c,a,d,n)    FlashWrite(dn, c,a,d,n)
#define FLASH_RD(dn, c,a,d,n)    FlashRead(dn, c,a,d,n)
#elif defined BOARD226
#define FLASH_WR(dn, c,a,d,n)    FLASH_WRITE(dn, c,a,d,n)
#define FLASH_RD(dn, c,a,d,n)    FLASH_READ(dn, c,a,d,n)
#else
#define FLASH_WR(dn,c,a,d,n)    FLASH_WRITE(dn,c,a,d,n)
#define FLASH_RD(dn,c,a,d,n)    FLASH_READ(dn,c,a,d,n)
#endif

#ifdef UCHAR
typedef unsigned char   UCHAR;
#endif
#ifdef USHORT
typedef unsigned short  USHORT;
#endif
#ifdef UINT
typedef unsigned int    UINT;
#endif
#ifdef ULONG
typedef unsigned long   ULONG;
#endif
#ifdef BOARD226
#include "slld_hal_226.h"
#endif

extern SLLD_STATUS slld_ReadOp
(
        BYTE  device_num,                     /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *target,                   /* variable in which to store read data */
        BYTECOUNT   len_in_bytes              /* number of bytes to read */

);

extern SLLD_STATUS slld_WriteOp
(
        BYTE  device_num,                     /* device number */
        ADDRESS     sys_addr,                 /* device address given by system */
        BYTE       *data_buf,                 /* variable containing data to program */
        BYTECOUNT   len_in_bytes,             /* number of bytes to program */
        DEVSTATUS  *dev_status_ptr            /* variable to store device status */
);
/*****************************************************************************/

extern SLLD_STATUS slld_GetDevNumFromAddr
(
        ADDRESS     sys_addr,                  /* device address given by system */
        BYTE  *device_num                        /* device number */
);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __INC_lldh */
