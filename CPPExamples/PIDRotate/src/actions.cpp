#include <actions.hpp>
#include <arpirobot/core/log/Logger.hpp>
#include <main.hpp>

using namespace arpirobot;

////////////////////////////////////////////////////////////////////////////////
/// JSDriveAction
////////////////////////////////////////////////////////////////////////////////

void JSDriveAction::begin(){
    // Only one action can lock a device at a time
    // The newest action keeps control of the device. 
    // Any other action that had previously locked the device will be stopped.
    // This ensures that only one action will ever attempt to control the motors at any given time
    lockDevices({Main::robot->flmotor, Main::robot->frmotor, 
            Main::robot->rlmotor, Main::robot->rrmotor});

    // Coast mode is more natural for human driving
    Main::robot->setBrakeMode(false);
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
    // This action will not stop on its own
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/// RotateAction
////////////////////////////////////////////////////////////////////////////////

RotateAction::RotateAction(double degrees) : degrees(degrees){

}

void RotateAction::begin(){
    // Only one action can lock a device at a time
    // The newest action keeps control of the device. 
    // Any other action that had previously locked the device will be stopped.
    // This ensures that only one action will ever attempt to control the motors at any given time
    lockDevices({Main::robot->flmotor, Main::robot->frmotor, 
            Main::robot->rlmotor, Main::robot->rrmotor});

    // Reset correct counter
    correctCounter = 0;

    // Brake mode helps prevent oscillations
    Main::robot->setBrakeMode(true);

    // Configure rotate PID
    Main::robot->rotatePid.reset();
    Main::robot->rotatePid.setSetpoint(Main::robot->imu.getGyroZ() + degrees);

    // No forward / reverse motion during this action
    Main::robot->driveHelper.updateSpeed(0);
}

void RotateAction::process(){
    // Calculate current output power given current angle
    double power = Main::robot->rotatePid.getOutput(Main::robot->imu.getGyroZ());

    // Update rotation based on pid output power
    Main::robot->driveHelper.updateRotation(-power);
}

void RotateAction::finish(bool wasInterrupted){

    // If this action is stopped, make sure the motors stop too
    Main::robot->driveHelper.update(0, 0);

    // Wait for 100ms to ensure brake mode has long enough to work
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool RotateAction::shouldContinue(){
    if(std::abs(Main::robot->imu.getGyroZ() - Main::robot->rotatePid.getSetpoint()) <= 3.0){
        // If within 3 degrees of target increment counter
        // 3 degree tolerance works well experimentally
        // Do not use too small of a tolerance or IMU drift may cause values
        correctCounter++;
    }else{
        // Reset counter when no longer correct angle
        correctCounter = 0;
    }

    // Stop if the angle has been correct for the past 10 iterations
    // Continue if counter less than 10 (should not stop)
    return correctCounter < 10;
}
