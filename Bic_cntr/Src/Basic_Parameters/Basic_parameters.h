/*
 * Basic_parameters.h
 *
 *  Created on: 11 cze 2017
 *      Author: Fero
 */

#ifndef BASIC_PARAMETERS_BASIC_PARAMETERS_H_
#define BASIC_PARAMETERS_BASIC_PARAMETERS_H_

typedef struct Basic_Parameter *p_BasicParamPtr;

void aggregate_basic_params(p_BasicParamPtr);
void evalVelocity(p_BasicParamPtr basParamPtr);
/**
 * Module responsible for handling basic parameters,
 * such as : time, distance, velocity
 */
typedef struct Basic_Parameter{
	///Wheel radius
	float radius_km;// = 0.5/1000;

	///Time related params
	char time_sec[3];
	char time_hrs[3];
	char time_min[3];

	short int t_secs;
	short int t_mins;
	short int t_hours;

	int t_secs_total;
	int t_mins_total;
	int t_hours_total;

	volatile int round_time_ms;
	///Velocity related params
	char velocity_string[6];
	char distance_string[15];

	int dist_total;

	void (*battlvlToStr)(p_BasicParamPtr);
	void (*evalVelocity)(p_BasicParamPtr);
	void (*aggrParams)(p_BasicParamPtr);
	void (*timeToStr) (p_BasicParamPtr);
	void (*evalDist) (p_BasicParamPtr);
	void (*tickTime) (p_BasicParamPtr);
	void (*resetBasParams) (p_BasicParamPtr);
} Basic_Parameter;


#endif /* BASIC_PARAMETERS_BASIC_PARAMETERS_H_ */
