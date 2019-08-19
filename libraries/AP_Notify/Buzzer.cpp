/*
  Buzzer driver
*/
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
#include "Buzzer.h"

#include <AP_HAL/AP_HAL.h>

#include "AP_Notify.h"

#ifndef HAL_BUZZER_ON
 #define HAL_BUZZER_ON 1
 #define HAL_BUZZER_OFF 0 
#endif



extern const AP_HAL::HAL& hal;


bool Buzzer::init()
{
    if (pNotify->buzzer_enabled() == false) {
        return false;
    }
#if defined(HAL_BUZZER_PIN)
    _pin = HAL_BUZZER_PIN;
#else
    _pin = pNotify->get_buzz_pin();
#endif
    if(!_pin) return false;

    // setup the pin and ensure it's off
    hal.gpio->pinMode(_pin, HAL_GPIO_OUTPUT);
    on(false);

    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    _flags.armed = AP_Notify::flags.armed;
    _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    _flags.failsafe_radio = false;
    _flags.was_armed = false;
    _flags.initialize_done = false;
    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer::update()
{
    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        play_pattern(SINGLE_BUZZ);
    }

    // reduce 50hz call down to 10hz
    _counter++;
    if (_counter < 5) {
        return;
    }
    _counter = 0;

    // complete currently played pattern
    if (_pattern != NONE) {
        _pattern_counter++;
        switch (_pattern) {
            case SINGLE_BUZZ:
                // buzz for 10th of a second
                if (_pattern_counter == 1) {
                    on(true);
                }else{
                    on(false);
                    _pattern = NONE;
                }
                return;
            case DOUBLE_BUZZ:
                // buzz for 10th of a second
                switch (_pattern_counter) {
                    case 1:
                        on(true);
                        break;
                    case 2:
                        on(false);
                        break;
                    case 3:
                        on(true);
                        break;
                    case 4:
                    default:
                        on(false);
                        _pattern = NONE;
                        break;
                }
                return;
            case ARMING_BUZZ:
                // record start time
                if (_pattern_counter == 1) {
                    _arming_buzz_start_ms = AP_HAL::millis();
                    on(true);
                } else {
                    // turn off buzzer after 3 seconds
                    if (AP_HAL::millis() - _arming_buzz_start_ms >= BUZZER_ARMING_BUZZ_MS) {
                        _arming_buzz_start_ms = 0;
                        on(false);
                        _pattern = NONE;
                    }
                }
                return;
            case BARO_GLITCH:
                // four fast tones
                switch (_pattern_counter) {
                    case 1:
                    case 3:
                    case 5:
                    case 7:
                    case 9:
                        on(true);
                        break;
                    case 2:
                    case 4:
                    case 6:
                    case 8:
                        on(false);
                        break;
                    case 10:
                        on(false);
                        _pattern = NONE;
                        break;
                    default:
                        // do nothing
                        break;
                }
                return;
            case EKF_BAD:
                // four tones getting shorter)
                switch (_pattern_counter) {
                    case 1:
                    case 5:
                    case 8:
                    case 10:
                        on(true);
                        break;
                    case 4:
                    case 7:
                    case 9:
                        on(false);
                        break;
                    case 11:
                        on(false);
                        _pattern = NONE;
                        break;
                    default:
                        // do nothing
                        break;
                }
                return;
            case RADIOLOST_BUZZ:
                // radio failsafe event (play short then long tone)
                if (_pattern_counter != 2) {
                    if (_pattern_counter >= 10) {
                        on(false);
                        // if system was (ever) armed then repeat indefinitely (for lost model locator)
                        if (_flags.was_armed) {
                            if (_pattern_counter >= 20) {
                                _pattern_counter = 0;
                            }
                        } else {
                            _pattern = NONE;
                        }
                    } else {
                        on(true);
                    }
                } else {
                    on(false);
                }
                break;
            case RADIOBACK_BUZZ:
                // radio recovered after failsafe (play long then short tone)
                if (_pattern_counter != 5) {
                    if (_pattern_counter >= 7) {
                        _pattern = NONE;
                        on(false);
                    } else {
                        on(true);
                    }
                } else {
                    on(false);
                }
                break;
            case INITIALIZE_BUZZ:
                // play tones to inidicate system ready
                switch (_pattern_counter) {
                    case 1:
                        on(true);
                        break;
                    case 2:
                        on(false);
                        break;
                    case 3:
                        on(true);
                        break;
                    case 4:
                        on(false);
                        break;
                    case 5:
                        on(true);
                        break;
                    case 6:
                    default:
                        on(false);
                        _pattern = NONE;
                        break;
                }
                break;
            default:
                // do nothing
                break;
        }
    }

    // check radio failsafe change
    if (_flags.failsafe_radio != AP_Notify::flags.failsafe_radio && !AP_Notify::flags.initialising) {
        _flags.failsafe_radio = AP_Notify::flags.failsafe_radio;
        // play tone to indicate radio status
        play_pattern(_flags.failsafe_radio ? RADIOLOST_BUZZ : RADIOBACK_BUZZ);
        return;
    }

    // check if need to signal system initialization complete
    if ((!_flags.initialize_done) && (!AP_Notify::flags.initialising)) {
        _flags.initialize_done = true;
        // play tone (only once) to indicate system ready
        play_pattern(INITIALIZE_BUZZ);
        return;
    }
    
    // if playing continous radio-failsafe tones then ignore other notifications
    if (_flags.failsafe_radio && _flags.was_armed) {
        return;
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            _flags.was_armed = true;
            play_pattern(ARMING_BUZZ);
        }else{
            // single buzz when disarmed
            play_pattern(SINGLE_BUZZ);
        }
        return;
    }

    // check ekf bad
    if (_flags.ekf_bad != AP_Notify::flags.ekf_bad) {
        _flags.ekf_bad = AP_Notify::flags.ekf_bad;
        if (_flags.ekf_bad) {
            // ekf bad warning buzz
            play_pattern(EKF_BAD);
        }
        return;
    }

    // if vehicle lost was enabled, starting beep
    if (AP_Notify::flags.vehicle_lost) {
        play_pattern(DOUBLE_BUZZ);
    }

    // if battery failsafe constantly single buzz
    if (AP_Notify::flags.failsafe_battery) {
        play_pattern(SINGLE_BUZZ);
    }
}

// on - turns the buzzer on or off
void Buzzer::on(bool turn_on)
{
    // return immediately if nothing to do
    if (_flags.on == turn_on) {
        return;
    }

    // update state
    _flags.on = turn_on;

    // pull pin high or low
    hal.gpio->write(_pin, _flags.on? HAL_BUZZER_ON : HAL_BUZZER_OFF);
}

/// play_pattern - plays the defined buzzer pattern
void Buzzer::play_pattern(BuzzerPattern pattern_id)
{
    _pattern = pattern_id;
    _pattern_counter = 0;
}

