#pragma once

#include <arpirobot/core/robot/BaseRobot.hpp>

#include <arpirobot/core/action/ActionSeries.hpp>

#include <arpirobot/core/drive/ArcadeDriveHelper.hpp>
#include <arpirobot/core/drive/CubicAxisTransform.hpp>
#include <arpirobot/core/drive/SquareRootAxisTransform.hpp>

#include <arpirobot/devices/gamepad/Gamepad.hpp>
#include <arpirobot/devices/gamepad/ButtonPressedTrigger.hpp>
#include <arpirobot/devices/adafruitmotorhat/AdafruitMotorHatMotor.hpp>
#include <arpirobot/devices/gpio/StatusLED.hpp>

#include <arpirobot/arduino/iface/ArduinoUartInterface.hpp>
#include <arpirobot/arduino/sensor/VoltageMonitor.hpp>
#include <arpirobot/arduino/sensor/Mpu6050Imu.hpp>
#include <arpirobot/arduino/sensor/SingleEncoder.hpp>

#include <actions.hpp>


using namespace arpirobot;


class Robot : public BaseRobot{
public:
    
    // Run when the robot starts
    void robotStarted();

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

    // Custom helper function to set the brake mode state of all motors
    void setBrakeMode(bool brakeMode);


    // Add devices and constants here as member objects
    // These should be public so actions can access them using Main::robot

    ////////////////////////////////////////////////////////////////////////////
    /// Devices
    ////////////////////////////////////////////////////////////////////////////

    // Motors
    AdafruitMotorHatMotor flmotor {3};
    AdafruitMotorHatMotor rlmotor {4};
    AdafruitMotorHatMotor frmotor {2};
    AdafruitMotorHatMotor rrmotor {1};

    // Drive helper
    ArcadeDriveHelper driveHelper {{flmotor, rlmotor}, {frmotor, rrmotor}};

    // Arduino interface
    ArduinoUartInterface arduino {"/dev/ttyUSB0", 57600};

    // Arduino sensors
    VoltageMonitor vmon {"A0", 5.00, 30000, 7500};
    SingleEncoder lencoder {2, false};
    SingleEncoder rencoder {3, false};
    Mpu6050Imu imu;

    // Status LED connected to raspberry pi GPIO #12
    StatusLED statusLed { 12 };

    ////////////////////////////////////////////////////////////////////////////
    /// Actions
    ////////////////////////////////////////////////////////////////////////////

    // Action instances
    JSDriveAction jsDriveAction;
    DriveDistanceAction driveTwoFeetAction { 0.6, 100, 3000 };
    RotateDegreesAction rotatePos90Action { 0.8, 90, 5000 };
    RotateDegreesAction rotateNeg270Action { 0.8, -270, 5000 };
    WaitTimeAction wait250Action { 250 };
    
    // Action series (sequences of actions to run)
    ActionSeries autoSequence {
        // This is a list of actions to run sequentially
        {
            driveTwoFeetAction,         // Drive 2 ft forward
            wait250Action,              // Wait a short amount of time so brake mode works
            rotatePos90Action,          // Rotate 90 degrees
            wait250Action,              // Wait a short amount of time so brake mode works
            driveTwoFeetAction,         // Drive 2 ft forward
            wait250Action,              // Wait a short amount of time so brake mode works
            rotateNeg270Action,         // Rotate -270 degrees
            wait250Action,              // Wait a short amount of time so brake mode works
            driveTwoFeetAction,         // Drive 2 ft forward
            wait250Action               // Wait a short amount of time so brake mode works
        },     

        // This action will run after the action series finishes     
        jsDriveAction                   // When done, drive with joysticks again
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Gamepad and controls
    ////////////////////////////////////////////////////////////////////////////

    // Axis numbers
    const int DRIVE_AXIS = 1;
    const int ROTATE_AXIS = 2;
    const int AUTO_BUTTON = 0;

    // Joystick values beteween -deadband and +deadband are treated as zero
    const double DEADBAND = 0.1;

    // Gamepads
    Gamepad gp0 {0};

    // Gamepad action triggers
    ButtonPressedTrigger autoTrigger { gp0, AUTO_BUTTON, autoSequence };
};