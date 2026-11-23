#ifndef __YEEP144_CONDUCTIVITY_H
#define __YEEP144_CONDUCTIVITY_H

void YEEP144_Conductivity_Init(void);
uint16_t YEEP144_Conductivity_GetReading(void);
float YEEP144_Conductivity_CalculateConductivity(uint16_t adcValue);

#endif
#endif
