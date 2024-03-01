/* Author: Hikaru Sugiura */

#include <stdint.h>
#include <cmath>
#include "utils.h"

double rad_diff(double past, double current)
{
    double diff = past - current;
    if (((past * current) < 0) && (fabs(diff) > M_PI))
    {                                                      // Crossed +PI/-PI[rad] border
        diff = M_PI - fabs(past) + (M_PI - fabs(current)); // - to +
        if ((past > 0) && (current < 0))
        { // Case of + to -
            diff = -diff;
        }
    }
    return diff;
}

uint8_t convert_speed_mps_to_kmph(float mps)
{
    float km_p_h = mps * 3.6; // m/s to km/h
    return km_p_h * 10; // WHEEL 60 means 6.0 km/h
}

uint8_t convert_acc_mpss_to_kmphs(float mpss)
{
    double kmh_p_s = mpss * 3.6; // m/ss to km/h/s
    return kmh_p_s  /(0.4 * (32.0f / 256.0f));
}

uint8_t convert_speed_radps_to_kmph(float tread, float radps)
{
    double mps = radps * tread / 2 * 2.0f;  // Half Tread * Turning Fix, Wheel Speed will be Max/2 in spin turning.
    return convert_speed_mps_to_kmph(mps);
}

uint8_t convert_acc_radpss_to_kmphs(float tread, float radpss)
{
    double mpss = radpss * tread / 2 * 2.0f; // Half Tread * Turning Fix
    return convert_acc_mpss_to_kmphs(mpss);
}

