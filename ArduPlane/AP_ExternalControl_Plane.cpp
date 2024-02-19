/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  Sets the target global position for a loiter point.
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}
bool AP_ExternalControl_Plane::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }
    const float yaw_rate_cds = isnan(yaw_rate_rads)? 0: degrees(yaw_rate_rads)*100;

    // Copter velocity is positive if aircraft is moving up which is opposite the incoming NED frame.
    Vector3f velocity_NEU_ms {
        linear_velocity.x,
        linear_velocity.y,
        -linear_velocity.z };
    Vector3f velocity_up_cms = velocity_NEU_ms * 100;
    plane.mode_guided.set_velocity(velocity_up_cms, false, 0, !isnan(yaw_rate_rads), yaw_rate_cds);
    return true;
}

bool AP_ExternalControl_Plane::ready_for_external_control()
{
    return plane.motors->armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
