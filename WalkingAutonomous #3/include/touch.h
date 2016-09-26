/*
 * touch.h
 *
 *  Created on: 10/6/2015
 *      Author: USUARIO
 */

#ifndef INCLUDE_TOUCH_H_
#define INCLUDE_TOUCH_H_

#ifdef __cplusplus
extern "C" {
#endif

void touch_initialize(void);
int touch_get(int touch_channel);


#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_TOUCH_H_ */
