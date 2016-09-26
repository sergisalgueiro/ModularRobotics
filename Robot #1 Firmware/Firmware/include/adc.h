#ifndef _ADC_HEADER
#define _ADC_HEADER

#ifdef __cplusplus
extern "C" {
#endif

void adc_initialize(void);
int adc_get(int adc_channel);

#ifdef __cplusplus
}
#endif

#endif
