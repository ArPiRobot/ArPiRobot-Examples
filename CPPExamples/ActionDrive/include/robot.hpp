#pragma once

#include <actions.hpp>

#include <arpirobot/core/robot/BaseRobot.hpp>

#include <arpirobot/core/drive/ArcadeDriveHelper.hpp>
#include <arpirobot/core/drive/CubicAxisTransform.hpp>
#include <arpirobot/core/drive/SquareRootAxisTransform.hpp>

#include <arpirobot/devices/gamepad/Gamepad.hpp>
#include <arpirobot/devices/adafruitmotorhat/AdafruitMotorHatMotor.hpp>

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


    // Add devices and constants here as member objects
    // These should be public so actions can access them using Main::robot

    // Motors
    AdafruitMotorHatMotor flmotor {3};
    AdafruitMotorHatMotor rlmotor {4};
    AdafruitMotorHatMotor frmotor {2};
    AdafruitMotorHatMotor rrmotor {1};

    // Drive helper. Takes a speed and rotation and calculates motor speeds.
    // Configure the drive helper to control all four drive motors
    ArcadeDriveHelper driveHelper {{&flmotor, &rlmotor}, {&frmotor, &rrmotor}};

    // Gamepads
    Gamepad gp0 {0};

    // Axis transforms
    CubicAxisTransform driveAxisTransform {0, 0.5};
    SquareRootAxisTransform rotateAxisTransform;

    // Axis numbers
    const int DRIVE_AXIS = 1;
    const int ROTATE_AXIS = 2;

    // Joystick values beteween -deadband and +deadband are treated as zero
    // A joystick will generally not read exactly zero, so treat anything small 
    // enough as a zero to prevent trying to move the motors very slowly
    const double DEADBAND = 0.1;

    // Action instances
    JSDriveAction jsDriveAction;

};