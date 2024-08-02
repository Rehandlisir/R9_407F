#include "./BSP/Exception_handling/exception_handling.h"
void ex_handl_Init(void)
{
	linearActuatordec_init();
}
void ex_handl_brake(void)
{
	if (LEFT_BREAK_STATE && (!RIGHT_BRAKE_STATE )) //LEFT push
	{
		g_slaveReg[4] = 1;
		g_slaveReg[27] = 1;
		// printf("push\n");
	}
    else if ((!LEFT_BREAK_STATE) && RIGHT_BRAKE_STATE ) //RIGHT push
	{
		g_slaveReg[4] = 1;
		g_slaveReg[27] = 1;

		// printf("push\n");

	}
	else if ((LEFT_BREAK_STATE) && RIGHT_BRAKE_STATE ) // LEFT && RIGHT push
	{
		g_slaveReg[4] = 1;
		g_slaveReg[27] = 1;
		// printf("push\n");
	}
	else //LEFT && RIGHT  DRIVE
	{
		g_slaveReg[4] = 1;
		g_slaveReg[27] = 1;
		// printf("drive\n");  
	}
		g_slaveReg[4] = 1;
		g_slaveReg[27] = 1;
}

void ex_handl_LRmoter(void)
{
	
	
	
	;
	
	
}
void linearActuatordec_init(void)
{
	;
}


void ex_handl_linearActuator(void)
{	
	CanCmdled(LED_YELLOW,LED_GREEN,LED_GREEN,LED_GREEN,LED_GREEN);
}

void ex_handl_battary(void)
{
	
	;
	
}

void ex_handl_joystic(void)
{
	
	
	;
	
}

void ex_handl_indicatorlight(void)
{
	
	;
}

void ex_handl_excute(void)
{
	
	ex_handl_brake();
	
	ex_handl_LRmoter();
	
	ex_handl_linearActuator();
	
	ex_handl_battary();
	
	ex_handl_joystic();
	
}
