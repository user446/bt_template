#include "packet.h"
#include "string.h"

static void SplitWord(short word, unsigned char container[2], bool invert)
{
	if(!invert)
	{
		container[0] = (unsigned char)((word&0xFF00)>>8);
		container[1] = (unsigned char)(word&0x00FF);
	}
	else
	{
		container[0] = (unsigned char)(word&0xFF00);
		container[1] = (unsigned char)((word&0x00FF)>>8);
	}
}
//

bool MakePacket(struct Command cmd, unsigned char result[128], unsigned char* data, unsigned short len)
{
	bool ret = true;
	
	if(len > cmd.len)
		ret = false;
	else
	{
		unsigned char counter = 0;
		
		result[counter] = cmd.prefix;
		counter++;
		
		unsigned char id_container[2] = {0};
		SplitWord(cmd.id, id_container, false);
		memcpy(result+counter, id_container, 2);
		counter+=2;
		
		result[counter++] = cmd.len;
		result[counter++] = cmd.type;
		
		unsigned char code_container[2] = {0};
		SplitWord(cmd.code, code_container, false);
		
		memcpy(result+counter, code_container, 2);
		counter+=2;
		
		memcpy(result+counter, data, len*sizeof(data[0]));
		counter+=len*sizeof(data[0]);
		
		unsigned char crc_container[2] = {0};
		SplitWord(ComputeCRC16(result, counter), crc_container, false);
		memcpy(result+counter, crc_container, 2);
	}
	return ret;
}
//

unsigned short ComputeCRC16(unsigned char* data, unsigned short len)
{
	unsigned char j;
	unsigned short reg_crc = 0xFFFF;
	
	while(len--)
	{
		reg_crc ^= *data++;
		for(j = 0; j < 8; j++)
		{
			if(reg_crc & 0x01) /*lsb(bit 0) = 1*/
				reg_crc = (reg_crc >> 1)^0xA001;
			else
				reg_crc = (reg_crc >> 1);
		}
	}
	return reg_crc;
}
//
