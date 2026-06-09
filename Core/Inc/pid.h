/* ***************************************************************** */
/* File name:        pid.h                                           */
/* File description: Header file containing the functions/methods    */
/*                   interfaces for handling the PID                 */
/* Author name:      julioalvesMS, IagoAF, rBacurau                  */
/* Creation date:    21jun2018                                       */
/* Revision date:    09nov2025                                       */
/* ***************************************************************** */

#ifndef SOURCES_CONTROLLER_PID_H_
#define SOURCES_CONTROLLER_PID_H_

#define UPDATE_PERIOD_MS      10
#define UPDATE_PERIOD         (UPDATE_PERIOD_MS/1000.0)
#define INTEGRATOR_MAX_SIZE   10

typedef struct pid_data_type {
	float fKp, fKi, fKd;         		// PID gains
	float fError_previous;       		// used in the derivative
	float fError_sum;            		// integrator cumulative error
	float fIntegratorSaturation;        // integrator saturation
	float fOutputSaturation;            // output saturation
} pid_data_type;


/* ************************************************ */
/* Method name:        vPidInit                     */
/* Method description: Initialize the PID controller*/
/* Input params:       xPidInstance: pid parameters   */
/* 					   float: fKp, fKi, fKd, fIntegratorSaturation, fOutputSaturation*/
/* Output params:      n/a                          */
/* ************************************************ */
void vPidInit(pid_data_type *xPidInstance, float fKp, float fKi, float fKd, float fIntegratorSaturation, float fOutputSaturation);


/* ************************************************** */
/* Method name:        vPidSetKp                      */
/* Method description: Set a new value for the PID    */
/*                     proportional constant          */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetKp(pid_data_type *xPidInstance, float fKp);


/* ************************************************** */
/* Method name:        fPidGetKp                      */
/* Method description: Get the value from the PID     */
/*                     proportional constant          */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: Value                   */
/* ************************************************** */
float fPidGetKp(pid_data_type *xPidInstance);


/* ************************************************** */
/* Method name:        vPidSetKi                      */
/* Method description: Set a new value for the PID    */
/*                     integrative constant           */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetKi(pid_data_type *xPidInstance, float fKi);


/* ************************************************** */
/* Method name:        fPidGetKi                      */
/* Method description: Get the value from the PID     */
/*                     integrative constant           */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: Value                   */
/* ************************************************** */
float fPidGetKi(pid_data_type *xPidInstance);


/* ************************************************** */
/* Method name:        vPidSetKd                      */
/* Method description: Set a new value for the PID    */
/*                     derivative constant            */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fKd: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetKd(pid_data_type *xPidInstance, float fKd);


/* ************************************************** */
/* Method name:        fPidGetKd                      */
/* Method description: Get the value from the PID     */
/*                     derivative constant            */
/* Input params:       xPidInstance: pid parameters   */
/* Output params:      float: Value                   */
/* ************************************************** */
float fPidGetKd(pid_data_type *xPidInstance);


/* ************************************************** */
/* Method name:        vPidSetIntegratorWindow        */
/* Method description: Set a new value for the        */
/*                     integrator saturation          */
/* Input params:       xPidInstance: pid parameters   */
/* 					   fIntegratorSaturation: New value */
/* Output params:      n/a                            */
/* ************************************************** */
void vPidSetIntegratorSaturation (pid_data_type *xPidInstance, float fIntegratorSaturation);


/* ************************************************** */
/* Method name:        usPidGetIntegratorWindow       */
/* Method description: Get the value from the         */
/*                     integrator saturation          */
/* Input params:       xPidInstance: pid parameters   */
/* 					   n/a                            */
/* Output params:      float: maximum integrator value*/
/* ************************************************** */
float fPidGetIntegratorSaturation (pid_data_type *xPidInstance);


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
float fPidUpdateData(pid_data_type *xPidInstance, float fSetValue, float fSensorValue);


#endif /* SOURCES_CONTROLLER_PID_H_ */
