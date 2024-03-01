#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>

double rad_diff(double past, double current); // Compares 2 radian values and returns difference between them.

uint8_t convert_speed_mps_to_kmph(float);
uint8_t convert_acc_mpss_to_kmphs(float);

uint8_t convert_speed_radps_to_kmph(float tread,float radps);
uint8_t convert_acc_radpss_to_kmphs(float tread, float radpss);

#endif // UTILS_H
