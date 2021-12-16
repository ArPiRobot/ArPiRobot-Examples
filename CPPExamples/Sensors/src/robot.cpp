#include <robot.hpp>

#include <arpirobot/core/log/Logger.hpp>
#include <arpirobot/core/action/ActionManager.hpp>
#include <arpirobot/core/network/NetworkTable.hpp>

#include <string>

using namespace arpirobot;


void Robot::robotStarted(){
    // Each sensor is instantiated, but not associated with an arduino yet.
    // Each sensor must be added to exactly one arduino interface.
    // A sensor should not be added to multiple arduinos
    // All sensors must be added before starting the arduino interface
    arduino.addDevice(&vmon);
    arduino.addDevice(&lencoder);
    arduino.addDevice(&rencoder);
    arduino.addDevice(&imu);
    arduino.addDevice(&usonic);
    arduino.addDevice(&ldetector);
    arduino.addDevice(&rdetector);

    // Start the arduino that this interface communicates with
    // Once begin has been called the arduino will start providing sensor data
    // However, no more devices can be added.
    // Before begin is called no sensor will receive any data, thus the 
    // values from the sensors are meaningless
    arduino.begin();

    // The main vmon will show the voltage in the drive station's battery indicator
    vmon.makeMainVmon();
}

void Robot::robotEnabled(){

}

void Robot::robotDisabled(){

}

void Robot::enabledPeriodic(){

}

void Robot::disabledPeriodic(){

}

void Robot::periodic(){
    // Do not remove this line or some devices will be disabled.
    feedWatchdog();

    // Put some sensor data in the network table
    // This can be viewed in the drive station
    NetworkTable::set("L Pos", std::to_string(lencoder.getPosition()));
    NetworkTable::set("R Pos", std::to_string(rencoder.getPosition()));
    NetworkTable::set("Gyro Z", std::to_string(imu.getGyroZ()));
    NetworkTable::set("Accel Y", std::to_string(imu.getAccelY()));
    NetworkTable::set("US Dist", std::to_string(usonic.getDistance()));
    NetworkTable::set("L Detect", ldetector.getDigitalValue() ? "true" : "false");
    NetworkTable::set("R Detect", rdetector.getDigitalValue() ? "true" : "false");
}