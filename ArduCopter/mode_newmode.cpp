#include "Copter.h"

/*
 * Init and run calls for new flight mode
 * Gongpei
 */
// stabilize_init - initialise stabilize controller
bool Copter::ModeNEWMODE::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
            // initialise waypoint and spline controller
            wp_nav->wp_and_spline_init();

            build_path(!copter.failsafe.terrain);
            climb_start();
            newm_state=NewM_InitialClimb;
            _state_complete=false;
            return true;
        }else{
            return false;
    }
}

void Copter::ModeNEWMODE::run(bool disarm_on_land)
{
    // check if we need to move to next state
    if (_state_complete) {
        switch (newm_state) {
        case NewM_InitialClimb:
            return_start();
            newm_state=NewM_ReturnHome;
            break;
        case NewM_ReturnHome:
            loiterathome_start();
            break;
        case NewM_LoiterAtHome:
            if (get_land() || copter.failsafe.radio) {
                land_start();
                newm_state=NewM_Land;
            }else{
                descent_start();
                newm_state=NewM_Descent;
            }
            break;
        case NewM_Descent:
            // do nothing
            break;
        case NewM_Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (newm_state) {

    case NewM_InitialClimb:
        climb_return_run();
        break;

    case NewM_ReturnHome:
        climb_return_run();
        break;

    case NewM_LoiterAtHome:
        loiterathome_run();
        break;

    case NewM_Descent:
        descent_run();
        break;

    case NewM_Land:
        land_run(disarm_on_land);
        break;
    }
}


// rtl_loiterathome_start - initialise return to home
void Copter::ModeNEWMODE::loiterathome_start()
{
    newm_state = NewM_LoiterAtHome;
    _state_complete = false;

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(auto_yaw.default_mode(true) != AUTO_YAW_HOLD) {
        auto_yaw.set_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }
}

void Copter::ModeNEWMODE::loiterathome_run()
{
    // set flight mode and simple mode setting
    if (set_mode(LOITER, MODE_REASON_TX_COMMAND))
    {
            // alert user to mode change (except if autopilot is just starting up)
            if (ap.initialised)
            {
                AP_Notify::events.user_mode_change = 1;
            }
    }
    else
    {
        // alert user to mode change failure
        AP_Notify::events.user_mode_change_failed = 1;
    }
    _state_complete=true;
}


uint32_t Copter::ModeNEWMODE::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t Copter::ModeNEWMODE::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}
