#ifndef _CRC16_H_
#define _CRC16_H_

#include "mbed.h"
  
unsigned short crc16_ccitt(const unsigned char* buf, int idx_start, int idx_end);

#endif /* _CRC16_H_ */