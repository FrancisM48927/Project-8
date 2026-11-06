/********************************************************************** 
* Author: F. MAILOM
* CPEG222 SSD Header, 9/21/26
* NucleoF466ZE CMSIS Initialize/Display SSD
**********************************************************************/
#ifndef SSD_H // Include guard which prevents contents from
#define SSD_H // accidentally being included multiple times

#include "stm32f4xx.h"

void SSD_Init(void);
void SSD_DisplayValue(uint16_t value);
void SSD_DisplayFrequency(uint16_t tenthsHz); // xxx.x format
void SSD_Refresh(void);

#endif