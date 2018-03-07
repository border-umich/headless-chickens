/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_Analog.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_Analog::AP_Proximity_Analog(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
    // initialize analog sources
    for (uint8_t i=0; i<PROXIMITY_MAX_ANALOG; i++) {
        source[i] = hal.analogin->channel(_state.pin[i]);
    }
}

// compute filtered distance (in meters) for a single sensor
float AP_Proximity_Analog::distance_m(uint8_t instance) {
    // cope with changed settings
    source[instance]->set_pin(state.pin[instance]);    
    float v = source[instance]->voltage_average();

    float dist_m = 0;
    AP_Proximity::Proximity_Function function = (AP_Proximity::Proximity_Function)state.function.get();
    float scaling = state.scaling;
    float offset = state.offset;
    float distance_max = state.distance_max;

    // convert voltage reading to distance
    switch (function) {
    case AP_Proximity::FUNCTION_LINEAR:
        dist_m = (v - offset) * scaling;
        break;

    case AP_Proximity::FUNCTION_INVERTED:
        dist_m = (offset - v) * scaling;
        break;

    case AP_Proximity::FUNCTION_HYPERBOLA:
        if (v <= offset) {
            dist_m = 0;
        }
        dist_m = scaling / (v - offset);
        if (isinf(dist_m) || dist_m > distance_max) {
            dist_m = distance_max;
        }
        break;

    case AP_Proximity::FUNCTION_POWER:
        if (v <= 0) {
            dist_m = distance_max;
        }
        else {
            dist_m = scaling * powf(v, offset);
            if (dist_m > distance_max) {
                dist_m = distance_max;
            }
        }
        break;
    }
    if (dist_m < 0) {
        dist_m = 0;
    }

    // filter the computed distance
    filter[instance].set_cutoff_frequency(state.cutoff_frequency);
    return filter[instance].apply(dist_m, 0.01);
}

// update the state of the sensor
void AP_Proximity_Analog::update(void)
{
    // look through all sensors
    for (uint8_t i=0; i<PROXIMITY_MAX_ANALOG; i++) {
        if (source[i] != nullptr && state.pin[i] != -1) {
            // check for horizontal sensors
            if (state.orientation[i] <= ROTATION_YAW_315) {
                uint8_t sector = (uint8_t)state.orientation[i];
                _angle[sector] = sector * 45;
                _distance[sector] = distance_m(i);
                _distance_valid[sector] = (_distance[sector] >= state.distance_min) && (_distance[sector] <= state.distance_max);
                _last_update_ms = AP_HAL::millis();
                update_boundary_for_sector(sector);
            }
            // check for upward facing sensor
            else if (state.orientation[i] == ROTATION_PITCH_90) {
                _distance_upward = distance_m(i);
                _last_upward_update_ms = AP_HAL::millis();
            }
        }
    }

    // check for timeout and set health status
    if ((_last_update_ms == 0) || (AP_HAL::millis() - _last_update_ms > PROXIMITY_ANALOG_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_Analog::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms != 0) && (AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_ANALOG_TIMEOUT_MS)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}
