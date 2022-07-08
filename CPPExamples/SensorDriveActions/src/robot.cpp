#include <robot.hpp>

#include <arpirobot/core/log/Logger.hpp>
#include <arpirobot/core/action/ActionManager.hpp>
#include <arpirobot/core/network/NetworkTable.hpp>

using namespace arpirobot;


void Robot::robotStarted(){

    Logger::logDebug("Motor Instance Addresses:");
    Logger::logDebug("flmotor = " + std::to_string((intptr_t)&flmotor));
    Logger::logDebug("frmotor = " + std::to_string((intptr_t)&frmotor));
    Logger::logDebug("rlmotor = " + std::to_string((intptr_t)&rlmotor));
    Logger::logDebug("rrmotor = " + std::to_string((intptr_t)&rrmotor));
    
    Logger::logNewline();

    Logger::logDebug("Action Instance Addresses");
    Logger::logDebug("--------------------------------------------------");
    Logger::logDebug("jsDriveAction = " + std::to_string((intptr_t)&jsDriveAction));
    Logger::logDebug("driveTwoFeetAction = " + std::to_string((intptr_t)&driveTwoFeetAction));
    Logger::logDebug("rotatePos90Action = " + std::to_string((intptr_t)&rotatePos90Action));
    Logger::logDebug("rotatePos120Action = " + std::to_string((intptr_t)&rotatePos120Action));
    Logger::logDebug("rotateNeg270Action = " + std::to_string((intptr_t)&rotateNeg270Action));
    Logger::logDebug("wait250Action = " + std::to_string((intptr_t)&wait250Action));
    Logger::logDebug("autoSequence = " + std::to_string((intptr_t)&autoSequence));
    Logger::logDebug("otherAutoSequence = " + std::to_string((intptr_t)&otherAutoSequence));

    // Setup axis transforms
    gp0.setAxisTransform(DRIVE_AXIS, std::make_shared<CubicAxisTransform>(0, 0.5));
    gp0.setAxisTransform(ROTATE_AXIS, std::make_shared<SquareRootAxisTransform>());

    // Fix motor directions (as needed, depends on wiring)
    flmotor.setInverted(true);
    frmotor.setInverted(true);

    // Setup sensors and start using arduino to acquire sensor data
    // Each sensor must be added to an arduino interface
    arduino.addDevice(vmon);
    arduino.addDevice(lencoder);
    arduino.addDevice(rencoder);
    arduino.addDevice(imu);

    // Once being is called, no more devices can be added
    arduino.begin();

    // Calibrate IMU to reduce drift
    // Robot must be stationary during calibration and negative Y axis must be down
    imu.calibrate(5);

    // Show motor battery voltage in DS
    vmon.makeMainVmon();
}

void Robot::robotEnabled(){
    // Allow auto sequence to be triggered when the robot is enabled
    ActionManager::addTrigger(autoTrigger);
    ActionManager::addTrigger(otherAutoTrigger);

    // Enable joystick drive (by starting the action) when the robot is enabled
    ActionManager::startAction(jsDriveAction);
}

void Robot::robotDisabled(){

    // Don't allow auto sequence to be triggered when the robot is disabled
    ActionManager::removeTrigger(autoTrigger);
    ActionManager::removeTrigger(otherAutoTrigger);

    // Disabling the robot also stops the auto routine
    // Forcefully stopping an ActionSeries will prevent it from running its finish action
    if(autoSequence.isRunning())
        ActionManager::stopAction(autoSequence);
    
    if(otherAutoSequence.isRunning())
        ActionManager::stopAction(otherAutoSequence);
    
    if(jsDriveAction.isRunning())
        ActionManager::stopAction(jsDriveAction);

    // Brake mode when the robot becomes disabled
    setBrakeMode(true);
}

void Robot::enabledPeriodic(){
    // Kill auto routines with a button
    if(gp0.getButton(KILL_AUTO_BUTTON)){
        if(autoSequence.isRunning())
            ActionManager::stopAction(autoSequence);
        if(otherAutoSequence.isRunning())
            ActionManager::stopAction(otherAutoSequence);
        if(!jsDriveAction.isRunning())
            ActionManager::startAction(jsDriveAction);
    }
}

void Robot::disabledPeriodic(){

}

void Robot::periodic(){
    // Put sensor data on the network table so it is easy to see any issues
    NetworkTable::set("Gyro Z", std::to_string(imu.getGyroZ()));
    NetworkTable::set("L Pos", std::to_string(lencoder.getPosition()));
    NetworkTable::set("R Pos", std::to_string(rencoder.getPosition()));

    // Do not remove this line or some devices will be disabled.
    feedWatchdog();
}

// Helper function to set all four motor's brake mode state
void Robot::setBrakeMode(bool brakeMode){
    flmotor.setBrakeMode(brakeMode);
    frmotor.setBrakeMode(brakeMode);
    rlmotor.setBrakeMode(brakeMode);
    rrmotor.setBrakeMode(brakeMode);
}