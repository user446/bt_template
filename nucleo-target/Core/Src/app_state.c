#include "app_state.h"

volatile unsigned char current_state = 0;

int Check_AppState(void)
{
	return current_state;
}
//

void Set_AppState(unsigned char state)
{
	current_state = state;
}
//

