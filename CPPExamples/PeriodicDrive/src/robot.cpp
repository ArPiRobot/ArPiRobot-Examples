#include <robot.hpp>
#include <actions.hpp>

#include <arpirobot/core/log/Logger.hpp>
#include <arpirobot/core/action/ActionManager.hpp>
#include <arpirobot/core/network/NetworkTable.hpp>

using namespace arpirobot;


void Robot::robotStarted(){
    // Setup axis transforms
    gp0.setAxisTransform(DRIVE_AXIS, std::make_shared<CubicAxisTransform>(0, 0.5));
    gp0.setAxisTransform(ROTATE_AXIS, std::make_shared<SquareRootAxisTransform>());

    // Fix motor directions (as needed, depends on wiring)
    flmotor.setInverted(true);
    frmotor.setInverted(true);
}

void Robot::robotEnabled(){
    // Disable brake mode so driving is more natural
    flmotor.setBrakeMode(false);
    frmotor.setBrakeMode(false);
    rlmotor.setBrakeMode(false);
    rrmotor.setBrakeMode(false);
}

void Robot::robotDisabled(){
    // Put motors in brake mode so they resist movement while robot is disabled
    flmotor.setBrakeMode(true);
    frmotor.setBrakeMode(true);
    rlmotor.setBrakeMode(true);
    rrmotor.setBrakeMode(true);
}

void Robot::enabledPeriodic(){

    // Get gamepad values for speed and rotation
    // Speed is multiplied by -1 because it is easiest to make positive = forward
    // however, most gamepads will have the up direction on a stick negative
    double speed = -1 * gp0.getAxis(DRIVE_AXIS, DEADBAND);
    double rotation = gp0.getAxis(ROTATE_AXIS, DEADBAND);

    // Use the speed and rotation to move the motors
    // The drive helper takes care of the math to calculate the left and right speeds
    // Then it sets the motor speeds
    driveHelper.update(speed, rotation);
}

void Robot::disabledPeriodic(){

}

void Robot::periodic(){
    // Do not remove this line or some devices will be disabled.
    feedWatchdog();
}