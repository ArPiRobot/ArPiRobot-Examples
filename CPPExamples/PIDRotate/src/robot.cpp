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

    // Populate initial PID paremeters on network table
    // These can be edited at runtime in the drive station
    NetworkTable::set(ROTATE_KP_KEY, std::to_string(rotatePid.getKp()));
    NetworkTable::set(ROTATE_KI_KEY, std::to_string(rotatePid.getKi()));
    NetworkTable::set(ROTATE_KD_KEY, std::to_string(rotatePid.getKd()));

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

    // Update IMU angle periodically
    NetworkTable::set(IMU_Z_ANGLE_KEY, std::to_string(imu.getGyroZ()));

    // Update target angle periodically (keep up to date)
    NetworkTable::set(ROTATE_TARGET_KEY, std::to_string(rotatePid.getSetpoint()));

    // Inidicate to user when rotate PID is active (via action)
    if(rotateAct.isRunning())
        NetworkTable::set(ROTATE_ACTIVE_KEY, "true");
    else
        NetworkTable::set(ROTATE_ACTIVE_KEY, "false");

    // Update PID values if changed by drive station
    // If changed to an invalid value (not a number)
    // The value will be overwritten with the old PID setting
    if(NetworkTable::changed(ROTATE_KP_KEY)){
        try{
            rotatePid.setKp(std::stod(NetworkTable::get(ROTATE_KP_KEY)));
        }catch(const std::invalid_argument &e){
            NetworkTable::set(ROTATE_KP_KEY, std::to_string(rotatePid.getKp()));
        }
    }
    if(NetworkTable::changed(ROTATE_KI_KEY)){
        try{
            rotatePid.setKi(std::stod(NetworkTable::get(ROTATE_KI_KEY)));
        }catch(const std::invalid_argument &e){
            NetworkTable::set(ROTATE_KI_KEY, std::to_string(rotatePid.getKi()));
        }
    }
    if(NetworkTable::changed(ROTATE_KD_KEY)){
        try{
            rotatePid.setKd(std::stod(NetworkTable::get(ROTATE_KD_KEY)));
        }catch(const std::invalid_argument &e){
            NetworkTable::set(ROTATE_KD_KEY, std::to_string(rotatePid.getKd()));
        }
    }
}

void Robot::setBrakeMode(bool brakeMode){
    flmotor.setBrakeMode(brakeMode);
    frmotor.setBrakeMode(brakeMode);
    rlmotor.setBrakeMode(brakeMode);
    rrmotor.setBrakeMode(brakeMode);
}
