/*
 * manager.h
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

#ifndef INC_MANAGER_MANAGER_H_
#define INC_MANAGER_MANAGER_H_

typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR,
    STATUS_USB_MODE,
    STATUS_NORMAL_MODE,
    STATUS_INITIALIZATION,
    STATUS_READING,
    STATUS_IDLE,
	BLACKBOX_ERROR,
	SENSORS_INIT_ERROR
}Zeus_Status;

Zeus_Status NAV_MAIN();

#endif /* INC_MANAGER_MANAGER_H_ */
