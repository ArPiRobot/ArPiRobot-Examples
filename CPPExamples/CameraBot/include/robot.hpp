#pragma once

#include <arpirobot/robot/BaseRobot.hpp>

#include <arpirobot/drive/ArcadeDriveHelper.hpp>
#include <arpirobot/drive/CubicAxisTransform.hpp>
#include <arpirobot/drive/SquareRootAxisTransform.hpp>

#include <arpirobot/device/gamepad/Gamepad.hpp>
#include <arpirobot/device/adafruitmotorhat/AdafruitMotorHatMotor.hpp>

#include <arpirobot/arduino/iface/ArduinoUartInterface.hpp>
#include <arpirobot/arduino/sensor/VoltageMonitor.hpp>

#include <arpirobot/camera/RpicamCamera.hpp>

using namespace arpirobot;


class Robot : public BaseRobot{
public:
    
    // Run when the robot program starts
    void robotStarted();

    // Run when the robot program stops
    void robotStopped();

    // Runs once each time the robot becomes enabled
    void robotEnabled();

    // Runs once each time the robot becomes disabled
    void robotDisabled();

    // Runs periodically while the robot is enabled
    void enabledPeriodic();

    // Runs periodically while the robot is disabled
    void disabledPeriodic();

    // Runs periodically (regardless of robot state)
    void periodic();


    // Add devices and constants here as member objects
    // These should be public so actions can access them using Main::robot

    // Motors
    AdafruitMotorHatMotor flmotor {3};
    AdafruitMotorHatMotor rlmotor {4};
    AdafruitMotorHatMotor frmotor {2};
    AdafruitMotorHatMotor rrmotor {1};

    // Drive helper. Takes a speed and rotation and calculates motor speeds.
    // Configure the drive helper to control all four drive motors
    ArcadeDriveHelper driveHelper {{flmotor, rlmotor}, {frmotor, rrmotor}};

    // Gamepads
    Gamepad gp0 {0};

    // Axis numbers
    const int DRIVE_AXIS = 1;
    const int ROTATE_AXIS = 2;

    // Joystick values beteween -deadband and +deadband are treated as zero
    // A joystick will generally not read exactly zero, so treat anything small 
    // enough as a zero to prevent trying to move the motors very slowly
    const double DEADBAND = 0.1;

    // Raspberry Pi Camera module
    RpicamCamera cam0 {"0"};

    // Object used to communicate with an arduino connected to the raspberry pi via UART (serial)
    // First argument is the serial port name. Typically this will either be /dev/ttyUSB0 or /dev/tty/ACM0
    // Occasionally, the number may be different, but usually only if multiple arduinos are connected
    // The second argument is the buad rate. This needs to match the value set in the 
    // arduino firmware flashed to the arduino. By default, this is 57600
    ArduinoUartInterface arduino {"/dev/ttyACM0", 57600};

    // Simple voltage divider used to measure battery voltage.
    // This is used for the cheap voltage "sensors" that can be found online
    // First argument is the pin name. This depends on how you connected it to the arduino
    // Second argument is the voltage of the arduino board (either 5V or 3.3V usually)
    // Third argument is the resistance from power to measurement point (R1)
    // Fourth is the resistance from gnd to the measurement point (R2)
    // These resistances depend on how the voltage divider is constructed
    // For the cheap "sensors" purchased online these are R1 = 30000, R2 = 7500
    VoltageMonitor vmon {"A0", 3.3, 30000, 7500};
    
};
