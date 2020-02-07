#ifndef __PACKET_H
#define __PACKET_H

#include "stdbool.h"

#define DEF_TYPE		(2u)
#define WRITE_TYPE	(3u)
#define READ_TYPE		(4u)
#define NOTE_TYPE		(5u)

#define DEF_CODE		(0u)

typedef struct Command
{
	unsigned char prefix;
	unsigned short id;
	unsigned char len;
	unsigned char type;
	unsigned short code;
}Command;
//

static const struct Command	IKM_ECG_TX						= {0xAE,	0x2020,	20,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ECG_LIVE					= {0xAE,	0x2021, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_DEVNAME						= {0xAE,	0x2022, 64,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_DEVNUMBER					= {0xAE,	0x2023, 64,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_NAME					= {0xAE,	0x2024, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_SNAME				= {0xAE,	0x2025, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_PNAME				= {0xAE,	0x2026, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_DATE					= {0xAE,	0x2027, 20,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_DOCTOR				= {0xAE,	0x2028, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_HOSPITAL			= {0xAE,	0x2029, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_COMMENT			= {0xAE,	0x202A, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_USER_PHONE				= {0xAE,	0x202B, 128,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_FVT_ENABLE				= {0xAE,	0x202C, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_FVT_ECG_REC				= {0xAE,	0x202D, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_FVT_INTERVAL			= {0xAE,	0x202E, 4,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_FVT_DURATION			= {0xAE,	0x202F, 4,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_VT_ENABLE					= {0xAE,	0x2030, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_VT_ECG_REC				= {0xAE,	0x2031, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_VT_PERIOD					= {0xAE,	0x2032, 4,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_VT_DURATION				= {0xAE,	0x2033, 2,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ASYST_ENABLE			= {0xAE,	0x2034, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ASYST_ECG_REC			= {0xAE,	0x2035, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ASYST_DURATION		= {0xAE,	0x2036, 4,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_BRADY_ENABLE			= {0xAE,	0x2037, 2,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_BRADY_ECG_REC			= {0xAE,	0x2038, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_BRADY_INTERVAL		= {0xAE,	0x2039, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_BRADY_DURATION		= {0xAE,	0x203A, 2,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ATAF_ENABLE				= {0xAE,	0x203B, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ATAF_ECG_REC			= {0xAE,	0x203C, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ATAF_INTERVAL			= {0xAE,	0x203D, 4,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_ATAF_ECG_LIMIT		= {0xAE,	0x203E, 2,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_SYMPTOM_ENABLE		= {0xAE,	0x203F, 1,	DEF_TYPE,	DEF_CODE};
static const struct Command	IKM_SYMPTOM_ECG_LIMIT	= {0xAE,	0x2040, 2,	DEF_TYPE,	DEF_CODE};
                                                     
static const struct Command MESSAGE_ERROR					=	{0xAE,	0xDEAD, 0,	DEF_TYPE,	DEF_CODE};

bool MakePacket(struct Command cmd, unsigned char result[128], unsigned char* data, unsigned short len);
unsigned short ComputeCRC16(unsigned char* data, unsigned short len);

#endif
//
