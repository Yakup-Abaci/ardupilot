
/*
  external control library for plane
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Common/Location.h>

class AP_ExternalControl_Plane : public AP_ExternalControl {
public:
    /*
        Sets the target global position for a loiter point.
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) override WARN_IF_UNUSED;
private:
    /*
      Return true if Copter is ready to handle external control data.
      Currently checks mode and arm states.
    */
    bool ready_for_external_control() WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
