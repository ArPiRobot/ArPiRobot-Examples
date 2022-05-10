#include <actions.hpp>
#include <arpirobot/core/log/Logger.hpp>
#include <main.hpp>

using namespace arpirobot;

////////////////////////////////////////////////////////////////////////////////
/// JSDriveAction
////////////////////////////////////////////////////////////////////////////////

void JSDriveAction::begin(){
    // This action needs exclusive control of motors
    lockDevices({Main::robot->flmotor, Main::robot->frmotor, 
            Main::robot->rlmotor, Main::robot->rrmotor});

    // Driving is more natural in coast mode
    Main::robot->setBrakeMode(false);
}

void JSDriveAction::process(){
    // Get gamepad values for speed and rotation
    double speed = -1 * Main::robot->gp0.getAxis(Main::robot->DRIVE_AXIS, Main::robot->DEADBAND);
    double rotation = Main::robot->gp0.getAxis(Main::robot->ROTATE_AXIS, Main::robot->DEADBAND);

    // Use the speed and rotation to move the motors
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


////////////////////////////////////////////////////////////////////////////////
/// DriveDistanceAction
////////////////////////////////////////////////////////////////////////////////

DriveDistanceAction::DriveDistanceAction(double driveSpeed, int encoderTicks, int timeoutMs) :
        driveSpeed(driveSpeed), encoderTicks(encoderTicks), timeoutMs(timeoutMs) {
    
}

void DriveDistanceAction::begin(){
    // This action needs exclusive control of motors
    lockDevices({Main::robot->flmotor, Main::robot->frmotor, 
            Main::robot->rlmotor, Main::robot->rrmotor});
    
    // Store the time this action started
    startTime = std::chrono::steady_clock::now();

    // Make the current encoder position zero
    Main::robot->lencoder.setPosition(0);
    Main::robot->rencoder.setPosition(0);

    // Brake mode is better here because the robot will roll less after stopping at the desired distance
    Main::robot->setBrakeMode(true);

    // Start driving
    Main::robot->driveHelper.update(driveSpeed, 0);
}

void DriveDistanceAction::process(){
    // Nothing to do here.
}

void DriveDistanceAction::finish(bool wasInterrupted){
    // When action is stopped, stop motors
    Main::robot->driveHelper.update(0, 0);
}

bool DriveDistanceAction::shouldContinue(){
    // If this action has run too long, stop this action
    auto now = std::chrono::steady_clock::now();
    if(std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() >= timeoutMs){
        return false;
    }

    // If the robot has driven far enough stop this action
    double avgEncPos = 
        (Main::robot->lencoder.getPosition() + Main::robot->rencoder.getPosition()) / 2.0;
    if(avgEncPos > encoderTicks){
        return false;
    }

    // Otherwise, keep driving
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/// WaitTimeAction
////////////////////////////////////////////////////////////////////////////////

WaitTimeAction::WaitTimeAction(int timeMs) : timeMs(timeMs) {

}

void WaitTimeAction::begin(){
    startTime = std::chrono::steady_clock::now();
}

void WaitTimeAction::process(){

}

void WaitTimeAction::finish(bool wasInterrupted){

}

bool WaitTimeAction::shouldContinue(){
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() < timeMs;
}


////////////////////////////////////////////////////////////////////////////////
/// RotateDegreesAction
////////////////////////////////////////////////////////////////////////////////

RotateDegreesAction::RotateDegreesAction(double rotationSpeed, double degrees, int timeoutMs) : 
        rotationSpeed(std::abs(rotationSpeed)), degrees(degrees), timeoutMs(timeoutMs) {
    
}

void RotateDegreesAction::begin(){  
    // This action needs exclusive control of motors
    lockDevices({Main::robot->flmotor, Main::robot->frmotor, 
            Main::robot->rlmotor, Main::robot->rrmotor});

    // Store start time and angle
    startTime = std::chrono::steady_clock::now();
    startDegrees = Main::robot->imu.getGyroZ();

    // Brake mode is better here because the robot will roll less after stopping at the desired distance
    Main::robot->setBrakeMode(true);

    // Rotate the correct direction
    if(degrees >= 0){
        Main::robot->driveHelper.update(0, -rotationSpeed);
    }else{
        Main::robot->driveHelper.update(0, rotationSpeed);
    }
}

void RotateDegreesAction::process(){
    // Nothing to do here
}

void RotateDegreesAction::finish(bool wasInterrupted){
    // Make sure motors stop when this action stops
    Main::robot->driveHelper.update(0, 0);
}

bool RotateDegreesAction::shouldContinue(){
    // If this action has run too long, stop this action
    auto now = std::chrono::steady_clock::now();
    if(std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() >= timeoutMs){
        return false;
    }

    // If the robot has rotated far enough stop this action
    if(degrees >= 0 && Main::robot->imu.getGyroZ() - startDegrees > degrees){
        return false;
    }else if(degrees < 0 && Main::robot->imu.getGyroZ() - startDegrees < degrees){
        return false;
    }

    // Otherwise, keep rotating
    return true;
}
