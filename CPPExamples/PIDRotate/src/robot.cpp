#include <robot.hpp>

#include <arpirobot/core/log/Logger.hpp>
#include <arpirobot/core/action/ActionManager.hpp>
#include <arpirobot/core/network/NetworkTable.hpp>

using namespace arpirobot;


void Robot::robotStarted(){
    // Setup arduino and IMU
    arduino.addDevice(&imu);
    arduino.begin();
    imu.calibrate(10);

    // Setup axis transforms
    gp0.setAxisTransform(DRIVE_AXIS, &driveAxisTransform);
    gp0.setAxisTransform(ROTATE_AXIS, &rotateAxisTransform);

    // Fix motor directions (as needed, depends on wiring)
    flmotor.setInverted(true);
    frmotor.setInverted(true);

    // Run slightly faster for better PID performance
    rotateAct.setProcessPeriodMs(20);

    // When button pressed rotate 90 degrees
    // The action will exit once rotation is done then
    // The rotate action kills js drive by locking the 
    // same devices. When rotate is done, js drive is 
    // restarted to allow more driving
    ActionManager::addTrigger(&rotateTrigger);
}

void Robot::robotEnabled(){
    if(!jsdriveAct.isRunning())
        ActionManager::startAction(&jsdriveAct);
}

void Robot::robotDisabled(){
    ActionManager::stopAction(&rotateAct);
    ActionManager::stopAction(&jsdriveAct);
}

void Robot::enabledPeriodic(){

}

void Robot::disabledPeriodic(){

}

void Robot::periodic(){
    // Do not remove this line or some devices will be disabled.
    feedWatchdog();
}

void Robot::setBrakeMode(bool brakeMode){
    flmotor.setBrakeMode(brakeMode);
    frmotor.setBrakeMode(brakeMode);
    rlmotor.setBrakeMode(brakeMode);
    rrmotor.setBrakeMode(brakeMode);
}
