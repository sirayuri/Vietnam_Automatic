#ifndef INC_CDC_PROTOCOL_H_
#define INC_CDC_PROTOCOL_H_

#include <stdint.h>

void CDC_Protocol_Task(void);
void CDC_Debug_Task(void);
void CDC_Protocol_Receive(uint8_t *buf, uint32_t len);
void CDC_Protocol_LowVoltageNotify(float voltage);
void CDC_Protocol_LimitNotify(float voltage);
void CDC_Protocol_Process(char *rx);


#endif