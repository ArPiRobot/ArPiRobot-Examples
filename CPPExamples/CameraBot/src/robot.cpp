#include <robot.hpp>
#include <actions.hpp>

#include <arpirobot/log/Logger.hpp>
#include <arpirobot/action/ActionManager.hpp>
#include <arpirobot/network/NetworkTable.hpp>

using namespace arpirobot;


void Robot::robotStarted(){
    // Setup axis transforms
    gp0.setAxisTransform(DRIVE_AXIS, std::make_shared<CubicAxisTransform>(0, 0.5));
    gp0.setAxisTransform(ROTATE_AXIS, std::make_shared<SquareRootAxisTransform>());

    // Fix motor directions (as needed, depends on wiring)
    flmotor.setInverted(true);
    frmotor.setInverted(true);

    // Each sensor is instantiated, but not associated with an arduino yet.
    // Each sensor must be added to exactly one arduino interface.
    // A sensor should not be added to multiple arduinos
    // All sensors must be added before starting the arduino interface
    arduino.addDevice(vmon);

    // Start the arduino that this interface communicates with
    // Once begin has been called the arduino will start providing sensor data
    // However, no more devices can be added.
    // Before begin is called no sensor will receive any data, thus the 
    // values from the sensors are meaningless
    arduino.begin();

    // The main vmon will show the voltage in the drive station's battery indicator
    vmon.makeMainVmon();

    // Start streaming camera live
    cam0.setCaptureMode("1024x768@30/1");
    // cam0.setCaptureMode("640x480@30/1");
    cam0.setExtraOption("rotation", "180");
    cam0.setExtraOption("gain", "30");
    cam0.setHwAccel(true, false, false);
    // cam0.setHwAccel(true, true, true);
    cam0.startStreamH264("cam0", 4096);
}

void Robot::robotStopped(){
    cam0.stopStream();
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