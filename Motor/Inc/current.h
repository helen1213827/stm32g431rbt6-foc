#ifndef CURRENT_H
#define CURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ADC_RESOLUTION 4096.0f
#define ADC_REF_VOLTAGE 3.3f

#define SHUNT_RESISTANCE 0.003f // 3 mOhm
#define CURRENT_GAIN 20.0f

#define CURRENT_SCALE (ADC_REF_VOLTAGE / ADC_RESOLUTION) / (SHUNT_RESISTANCE * CURRENT_GAIN)

typedef struct
{
    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t raw_c;

    uint16_t offset_a;
    uint16_t offset_b;
    uint16_t offset_c;

    float ia;
    float ib;
    float ic;

} Current_t;

extern Current_t current;

void Current_Init(void);

void Current_CalibrateOffset(void);

void Current_Update(void);


#ifdef __cplusplus
}
#endif




#endif // CURRENT_H
