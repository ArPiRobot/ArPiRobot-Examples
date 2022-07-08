#include <actions.hpp>
#include <arpirobot/core/log/Logger.hpp>
#include <main.hpp>

using namespace arpirobot;


////////////////////////////////////////////////////////////////////////////////
/// JSDriveAction
////////////////////////////////////////////////////////////////////////////////

LockedDeviceList JSDriveAction::lockedDevices(){
    // Only one action can lock a device at a time
    // The newest action keeps control of the device. 
    // Any other action that had previously locked the device will be stopped.
    // This ensures that only one action will ever attempt to control the motors at any given time
    return { Main::robot->flmotor, Main::robot->frmotor, 
            Main::robot->rlmotor, Main::robot->rrmotor };
}

void JSDriveAction::begin(){
    
}

void JSDriveAction::process(){
    // Get gamepad values for speed and rotation
    // Speed is multiplied by -1 because it is easiest to make positive = forward
    // however, most gamepads will have the up direction on a stick negative
    double speed = -1 * Main::robot->gp0.getAxis(Main::robot->DRIVE_AXIS, Main::robot->DEADBAND);
    double rotation = Main::robot->gp0.getAxis(Main::robot->ROTATE_AXIS, Main::robot->DEADBAND);

    // Use the speed and rotation to move the motors
    // The drive helper takes care of the math to calculate the left and right speeds
    // Then it sets the motor speeds
    Main::robot->driveHelper.update(speed, rotation);
}

void JSDriveAction::finish(bool wasInterrupted){
    // If this action is stopped, make sure the motors stop too
    Main::robot->driveHelper.update(0, 0);
}

bool JSDriveAction::shouldContinue(){
    // This action will not stop on itself
    return true;
}
