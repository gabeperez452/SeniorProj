#include <stdio.h>
#include <stdbool.h>

int TOT_STATES=6;
static char signal = 'aaa';

typedef void (*StateFunction)();

void state_idle();
void state_checkHeadPosition();
void state_checkBedPosition();
void state_linearActuatorGoUp();
void state_checkVertScaperLimSwitch();
void state_scraperArmGoDown();
void state_checkHorizScraperArmLimSwitch();
void state_scraperArmGoSwoosh();
void state_scraperArmGoUp();
void state_error();

typedef enum
{
	NO_ERROR,
	CALIBRATION_ERROR
} ErrorCode;

ErrorCode error_code = NO_ERROR;

bool detect_error()
{
	//can simulate error with this
}

StateFunction current_state = state_idle; //initial state

int main()
{
	for (int i = 0; i < TOT_STATES; i++)
	{
		current_state();
	}
	return 0;
}

void state_idle()
{
	//raspberry pi signal is recieved, check for signal
	if (gpioListener() == 1)
	{
		current_state = state_checkHeadPosition()
	}
}

bool gpioListener()
{
	if(button is high)
	return 1;
	else return 0;
}