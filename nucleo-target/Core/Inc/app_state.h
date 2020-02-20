#ifndef __APP_STATE__
#define __APP_STATE__

#define STATE_REC_COMMAND 	(0u)
#define STATE_READ_DATA			(1u)
#define STATE_SEND_DATA			(2u)

int Check_AppState(void);
void Set_AppState(unsigned char state);

#endif
//
