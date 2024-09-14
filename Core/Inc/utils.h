/*
 * utils.h
 *
 *  Created on: Jul 24, 2024
 *      Author: hung
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))


#define DEVICE_UID_LEN (24)

void Read_Device_Uid(char *id);

#endif /* INC_UTILS_H_ */
