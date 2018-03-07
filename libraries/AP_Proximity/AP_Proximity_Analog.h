#pragma once

#include <Filter/Filter.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_ANALOG_TIMEOUT_MS 200 // requests timeout after 0.2 seconds

class AP_Proximity_Analog : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_Analog(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void);

    // get maximum and minimum distances (in meters) of sensor
    float distance_min() const { return state.distance_min; }
    float distance_max() const { return state.distance_max; };

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const;

private:

    // compute filtered distance (in meters) for a single sensor
    float distance_m(uint8_t instance);

    // analog sources and low-pass filters
    AP_HAL::AnalogSource *source[PROXIMITY_MAX_ANALOG];
    LowPassFilterFloat filter[PROXIMITY_MAX_ANALOG];

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last reading

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update distance
    float _distance_upward;             // upward distance in meters
};
