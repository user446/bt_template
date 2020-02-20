#ifndef __PACKET_H
#define __PACKET_H

#include "stdbool.h"

#define TYPE_DEF		(2u)
#define TYPE_WRITE	(3u)
#define TYPE_READ		(4u)
#define TYPE_NOTE		(5u)

#define DEF_CODE		(0u)

#define COMMANDS_AMOUNT (33u)
#define TYPES_AMOUNT (4u)
#define MIN_PACKET_LEN (8u)

typedef struct Command
{
	unsigned char 	prefix;
	unsigned short 	id;
	unsigned char 	len;
	unsigned char 	type;
	unsigned short 	code;
}Command;
//

static const unsigned char TYPEs[TYPES_AMOUNT] = {TYPE_DEF, TYPE_WRITE, TYPE_READ, TYPE_NOTE};

static const unsigned short COMMAND_IDs[COMMANDS_AMOUNT] = {0x2020,0x2021,0x2022,0x2023,0x2024,0x2025,0x2026,0x2027,0x2028,	
0x2029,0x202A,0x202B,0x202C,0x202D,0x202E,0x202F,0x2030,0x2031,0x2032,0x2033,0x2034,0x2035,0x2036,0x2037,0x2038,	
0x2039,0x203A,0x203B,0x203C,0x203D,0x203E,0x203F,0x2040};

static const struct Command	IKM_ECG_TX						= {0xAE,	0x2020,	128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ECG_LIVE					= {0xAE,	0x2021, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_DEVNAME						= {0xAE,	0x2022, 64,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_DEVNUMBER					= {0xAE,	0x2023, 64,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_NAME					= {0xAE,	0x2024, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_SNAME				= {0xAE,	0x2025, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_PNAME				= {0xAE,	0x2026, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_DATE					= {0xAE,	0x2027, 20,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_DOCTOR				= {0xAE,	0x2028, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_HOSPITAL			= {0xAE,	0x2029, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_COMMENT			= {0xAE,	0x202A, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_USER_PHONE				= {0xAE,	0x202B, 128,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_FVT_ENABLE				= {0xAE,	0x202C, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_FVT_ECG_REC				= {0xAE,	0x202D, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_FVT_INTERVAL			= {0xAE,	0x202E, 4,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_FVT_DURATION			= {0xAE,	0x202F, 4,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_VT_ENABLE					= {0xAE,	0x2030, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_VT_ECG_REC				= {0xAE,	0x2031, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_VT_PERIOD					= {0xAE,	0x2032, 4,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_VT_DURATION				= {0xAE,	0x2033, 2,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ASYST_ENABLE			= {0xAE,	0x2034, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ASYST_ECG_REC			= {0xAE,	0x2035, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ASYST_DURATION		= {0xAE,	0x2036, 4,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_BRADY_ENABLE			= {0xAE,	0x2037, 2,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_BRADY_ECG_REC			= {0xAE,	0x2038, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_BRADY_INTERVAL		= {0xAE,	0x2039, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_BRADY_DURATION		= {0xAE,	0x203A, 2,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ATAF_ENABLE				= {0xAE,	0x203B, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ATAF_ECG_REC			= {0xAE,	0x203C, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ATAF_INTERVAL			= {0xAE,	0x203D, 4,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_ATAF_ECG_LIMIT		= {0xAE,	0x203E, 2,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_SYMPTOM_ENABLE		= {0xAE,	0x203F, 1,	TYPE_DEF,	DEF_CODE};
static const struct Command	IKM_SYMPTOM_ECG_LIMIT	= {0xAE,	0x2040, 2,	TYPE_DEF,	DEF_CODE};
                                                     
static const struct Command MESSAGE_ERROR					=	{0xAE,	0xDEAD, 0,	TYPE_DEF,	DEF_CODE};

int MakePacket(struct Command cmd, unsigned char result[128], unsigned char* data, unsigned char len);
bool CheckIfPacket(unsigned char* input);
unsigned short ComputeCRC16(unsigned char* data, unsigned short len);

#endif
//
