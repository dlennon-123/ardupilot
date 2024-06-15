#include "Rover.h"

bool ModeHeadingLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

void ModeHeadingLoiter::update()
{
    Vector2f _distance_to_destination_loiter = rover.current_loc.get_distance_NE(_destination);

    const float loiter_radius = g2.loit_radius;
    // 0 turn rate is no limit
    float turn_rate = 0.0;
    float _desired_throttle = (_distance_to_destination_loiter.y - loiter_radius) * g2.loiter_speed_gain;
    float _desired_lateral = (_distance_to_destination_loiter.x - loiter_radius) * g2.loiter_speed_gain;

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_throttle, true);
    g2.motors.set_lateral(_desired_lateral);
}

// get desired location
bool ModeHeadingLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
