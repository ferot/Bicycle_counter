/*
 * Basic_Parameters.c
 *
 *  Created on: 11 cze 2017
 *      Author: Fero
 */
#include "Basic_parameters.h"

void conv_float_to_str(float value, char *tab);
void eval_velocity(p_BasicParamPtr basParamPtr);
void eval_total_dist(p_BasicParamPtr basParamPtr);
void aggregate_basic_params(p_BasicParamPtr basParamPtr);
void time_to_string(p_BasicParamPtr ptr);
void tick_time(p_BasicParamPtr ptr);
void reset_basic_params(p_BasicParamPtr basParamPtr);

Basic_Parameter BasParamMod = {
		.radius_km = 0.5/1000,
		.evalVelocity = eval_velocity,
		.evalDist = eval_total_dist,
		.aggrParams = aggregate_basic_params,
		.timeToStr = time_to_string,
		.tickTime = tick_time,
		.resetBasParams = reset_basic_params
};

/**
 * Function responsible for evaluating total distance
 * As a result - replaces new value in distance_string
 * @param basParamPtr - interface to Battery_Control module
 */
void eval_total_dist(p_BasicParamPtr basParamPtr) {

	float rad = basParamPtr->radius_km;
	int tot_dist = basParamPtr->dist_total;

	float dist = 2 * 3.14 * rad * tot_dist;
	conv_float_to_str(dist, basParamPtr->distance_string);
}

/**
 * Function responsible for evaluating velocity based on time of wheel round
 * As a result - replaces new value in velocity_string
 * @param basParamPtr - interface to Battery_Control module
 */
void eval_velocity(p_BasicParamPtr basParamPtr) {

	float time_hrs = (basParamPtr->round_time_ms / 1000.0) / 3600.0;
	float rad = basParamPtr->radius_km;

	float velocity = 2 * 3.14 * rad / time_hrs;

	conv_float_to_str(velocity, basParamPtr->velocity_string);
}

/**
 * Function implements time tick in mean of clock device
 * @param basParamPtr - interface to Battery_Control module
 */
inline void tick_time(p_BasicParamPtr ptr) {

	ptr->t_secs++;
	if (ptr->t_secs > 59) {
		ptr->t_secs = 0;
		ptr->t_mins++;
		if (ptr->t_mins > 59) {
			ptr->t_mins = 0;
			ptr->t_hours++;
			if (ptr->t_hours > 59) {
				ptr->t_hours = 0;
				//TODO: Handle days (maybe if overflowed -> dump entry into disk ?)
			}
		}
	}

}

/**
 * Function converts values into related strings
 * @param basParamPtr - interface to Battery_Control module
 */
inline void time_to_string(p_BasicParamPtr ptr) {
	itoa(ptr->t_secs, ptr->time_sec, 10);
	itoa(ptr->t_mins, ptr->time_min, 10);
	itoa(ptr->t_hours, ptr->time_hrs, 10);
}

/**
 * Function converting value to format XX,XX
 * Extracts parts and integrates it into single string
 * @param value to be converted
 * @param pointer to result-string
 */
void conv_float_to_str(float value, char *tab) {
	char buf2[4] = {0};
	char buf[16] = {0};
	char result[16] = {0};

	int decimal_part = ((int) (value * 100.0)) % 100;
	itoa((int) value, buf, 10);
	itoa(decimal_part, buf2, 10);

	strncpy(result, buf, 2);
	strncat(result, ",", 1);
	strncat(result, buf2, 2);
	strcpy(tab, result);
}

/**
 * Function responsible for aggregatting all important values
 * @param basParamPtr - interface to Battery_Control module
 */
void aggregate_basic_params(p_BasicParamPtr basParamPtr) {

	basParamPtr->t_secs_total += basParamPtr->t_secs;
	basParamPtr->t_mins_total += basParamPtr->t_mins;
	basParamPtr->t_hours_total += basParamPtr->t_hours;
	basParamPtr->dist_total++;

}

/**
 * Function zeroes values of time measurement and clears velocity string
 * @param basParamPtr - interface to Battery_Control module
 */
void reset_basic_params(p_BasicParamPtr basParamPtr) {
	basParamPtr->t_secs = 0;
	basParamPtr->t_mins = 0;
	basParamPtr->t_hours = 0;
	strcpy(basParamPtr->velocity_string, "0");
}
