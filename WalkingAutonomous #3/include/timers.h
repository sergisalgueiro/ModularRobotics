/*
 * timers.h
 *
 *  Created on: 11/6/2015
 *      Author: USUARIO
 */

#ifndef INCLUDE_TIMERS_H_
#define INCLUDE_TIMERS_H_

#ifdef __cplusplus
extern "C" {
#endif

void timer10ms_initialize(void);
void timer100ms_initialize(void);
int get_time(int time);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_TIMERS_H_ */
