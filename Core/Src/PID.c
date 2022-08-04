#include "PID.h"
/******** PID control algorithm ****************
 * All PID gains, Kp, Ki, Kd are scaled to 1000 magnitude, i.e, Kp = 10 here means Kp = 0.1 of real value
 * So the final output of the PID should be divided by 1000
 ***********************************************/

int32_t controlPID(PID_TypeDef *PID, int32_t setValue, int32_t currentValue){

	static int32_t outputPID, outputD, outputD_1, outputD_2;

	PID->error_k = setValue - currentValue;

	if(PID->filterEnableFlag != 0){
		outputD = PID->kd * ((PID->error_k - PID->error_k_1) + PID->filterCoef * (outputD_1 - outputD_2));
		outputPID = ((PID->kp * (PID->error_k - PID->error_k_1)) + (PID->ki * PID->error_k) + outputD) / 1000;
	}else{
		outputPID = ((PID->kp * (PID->error_k - PID->error_k_1)) + (PID->ki * PID->error_k) + (PID->kd * (PID->error_k - PID->error_k_2 + 2 * PID->error_k_1))) / 1000;
	}

	// Update parameters
	PID -> error_k_1 = PID -> error_k;
	PID -> error_k_2 = PID -> error_k_1;
	outputD_1 = outputD;
	outputD_2 = outputD_1;

	if(outputPID > PID->outputMAX) outputPID = PID->outputMAX;
	if(outputPID < PID->outputMIN) outputPID = PID->outputMIN;

	return outputPID;

}

/*********** Coef list *******************
 * Kp, Ki, Kd, FilterEnable, FilterCoef, OutMax, OutMin
 *****************************************/
void initPID(PID_TypeDef *PID, int32_t Kp, int32_t Ki, int32_t Kd, uint8_t FilterEnable, int32_t FilterCoef, int32_t OutMin, int32_t OutMax){
	PID->kp = Kp;
	PID->ki = Ki;
	PID->kd = Kd;
	PID->filterEnableFlag = FilterEnable;
	PID->filterCoef = FilterCoef;

	PID->error_k = 0;
	PID->error_k_1 = 0;
	PID->error_k_2 = 0;

	PID->setPoint = 0;

	PID->outputMAX = OutMax;
	PID->outputMIN = OutMin;
}
