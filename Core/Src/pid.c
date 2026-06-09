/* ***************************************************************** */
/* File name:        pid.c                                           */
/* File description: This file has a couple of useful functions to   */
/*                   control the implemented PID controller          */
/* Author name:      julioalvesMS, IagoAF, rBacurau                  */
/* Creation date:    21jun2018                                       */
/* Revision date:    09nov2025                                       */
/* ***************************************************************** */

#include "pid.h"
#include "main.h"

/* ************************************************ */
/* Method name:        vPidInit                     */
/* Method description: Initialize the PID controller*/
/* Input params:       xPidInstance: pid parameters   */
/* 					   float: fKp, fKi, fKd, fIntegratorSaturation, fOutputSaturation*/
/* Output params:      n/a                          */
/* ************************************************ */
void vPidInit(pid_data_type *xPidInstance, float fKp, float fKi, float fKd, float fIntegratorSaturation, float fOutputSaturation)
{
	xPidInstance->fKp = fKp;
	xPidInstance->fKd = fKd;
	xPidInstance->fKi = fKi;
	xPidInstance->fError_previous = 0;
	xPidInstance->fError_sum = 0.0;
	xPidInstance->fIntegratorSaturation = fIntegratorSaturation;
	xPidInstance->fOutputSaturation = fOutputSaturation;
}

/* ************************************************** */
/* Method name:        vPidSetKp                      */
/* Method description: Set a new value for the PID    */
/*                     proportional constant          */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fKp: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetKp(pid_data_type *xPidInstance, float fKp)
{
	xPidInstance->fKp = fKp;
}


/* ************************************************** */
/* Method name:        fPidGetKp                      */
/* Method description: Get the value from the PID     */
/*                     proportional constant          */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: Value                   */
/* ************************************************** */
float fPidGetKp(pid_data_type *xPidInstance)
{
	return xPidInstance->fKp;
}


/* ************************************************** */
/* Method name:        vPidSetKi                      */
/* Method description: Set a new value for the PID    */
/*                     integrative constant           */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fKi: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetKi(pid_data_type *xPidInstance, float fKi)
{
	xPidInstance->fKi = fKi;
}


/* ************************************************** */
/* Method name:        fPidGetKi                      */
/* Method description: Get the value from the PID     */
/*                     integrative constant           */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: Value                   */
/* ************************************************** */
float fPidGetKi(pid_data_type *xPidInstance)
{
	return xPidInstance->fKi;
}


/* ************************************************** */
/* Method name:        vPidSetKd                      */
/* Method description: Set a new value for the PID    */
/*                     derivative constant            */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fKd: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetKd(pid_data_type *xPidInstance, float fKd)
{
	xPidInstance->fKd = fKd;
}


/* ************************************************** */
/* Method name:        fPidGetKd                      */
/* Method description: Get the value from the PID     */
/*                     derivative constant            */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: Value                   */
/* ************************************************** */
float fPidGetKd(pid_data_type *xPidInstance)
{
	return xPidInstance->fKd;
}

/* ************************************************** */
/* Method name:        vPidSetIntegratorWindow        */
/* Method description: Set a new value for the        */
/*                     integrator saturation          */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fIntegratorSaturation: New value */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetIntegratorSaturation (pid_data_type *xPidInstance, float fIntegratorSaturation)
{
	xPidInstance->fIntegratorSaturation = fIntegratorSaturation;
}

/* ************************************************** */
/* Method name:        usPidGetIntegratorWindow       */
/* Method description: Get the value from the         */
/*                     integrator saturation          */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: maximum integrator value*/
/* ************************************************** */
float fPidGetIntegratorSaturation (pid_data_type *xPidInstance)
{
	return (xPidInstance->fIntegratorSaturation);
}

/* ************************************************** */
/* Method name:        fPidUpdateData                 */
/* Method description: Update the control output      */
/*                     using the reference and sensor */
/*                     value                          */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fSensorValue: Value read from  */
/*                     the sensor                     */
/*                     fReferenceValue: Value used as */
/*                     control reference              */
/* Output params:      float: New Control effort      */
/* ************************************************** */
float fPidUpdateData(pid_data_type *xPidInstance, float fSetValue, float fSensorValue)
{
	float fError, fDifference, fOut;

	// Proportional error
	fError = fSetValue - fSensorValue;

	//Ingtegral error
	xPidInstance->fError_sum += fError;

	// Integrator Saturation
	if(xPidInstance->fError_sum  > xPidInstance->fIntegratorSaturation)
		xPidInstance->fError_sum = xPidInstance->fIntegratorSaturation;
	else
		if(xPidInstance->fError_sum  < -1*xPidInstance->fIntegratorSaturation)
				xPidInstance->fError_sum = -1*xPidInstance->fIntegratorSaturation;

	// Differential error
	fDifference = (fError - xPidInstance->fError_previous);

	fOut = xPidInstance->fKp * fError
		 + xPidInstance->fKi * xPidInstance->fError_sum * UPDATE_PERIOD
		 + xPidInstance->fKd * fDifference / UPDATE_PERIOD;

	// Updates the last error
	xPidInstance->fError_previous = fError;

	// Output Saturation
	if(fOut > xPidInstance->fOutputSaturation)
		fOut = xPidInstance->fOutputSaturation;
	else 
		if (fOut < 0)
			fOut = 0;
	return fOut;
}
