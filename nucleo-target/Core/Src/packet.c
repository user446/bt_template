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

int MakePacket(struct Command cmd, unsigned char result[128], unsigned char* data, unsigned char len)
{
	int ret = 0;
	
	if(len + MIN_PACKET_LEN > cmd.len)
		ret = 0;
	else
	{
		unsigned char counter = 0;
		
		//префикс
		result[counter] = cmd.prefix;
		counter++;
		
		//id
		unsigned char id_container[2] = {0};
		SplitWord(cmd.id, id_container, false);
		memcpy(result+counter, id_container, 2);
		counter+=2;
		
		//длинна
		result[counter++] = cmd.len;
		//тип
		result[counter++] = cmd.type;
		
		//код сообщения
		unsigned char code_container[2] = {0};
		SplitWord(cmd.code, code_container, false);
		
		memcpy(result+counter, code_container, 2);
		counter+=2;
		
		//сообщение
		memcpy(result+counter, data, len*sizeof(data[0]));
		counter+=len*sizeof(data[0]);
		
		unsigned char crc_container[2] = {0};
		SplitWord(ComputeCRC16(result, counter), crc_container, false);
		memcpy(result+counter, crc_container, 2);
		
		ret = len + MIN_PACKET_LEN;
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

bool CheckIfPacket(unsigned char* input)
{
	bool ret = false;
	if(input[0] != 0xAE)
		return ret;
	else
	{
		if(input[1] != 0x20)
			return ret;
		else
		{
			int i = 0;
			for(i = 0; i < COMMANDS_AMOUNT; i++)
			{
				if(input[2] == (COMMAND_IDs[i]&0x00FF))
				{
					ret = true;
					break;
				}
			}
			if(i == COMMANDS_AMOUNT && ret == false)
				return ret;
			else
			{
				i = 0;
				for(int i = 0; i < TYPES_AMOUNT; i++)
				{
					if(TYPEs[i] == input[4])
					{
						ret = true;
						break;
					}
				}
				if(i == TYPES_AMOUNT && ret == false)
				{
					return ret;
				}
			}
		}
	}
	return ret;
}
//
