#pragma once

#include <arpirobot/core/robot/BaseRobot.hpp>

#include <actions.hpp>

// Other includes (for devices and other objects) here
#include <arpirobot/core/control/PID.hpp>
#include <arpirobot/core/action/ActionSeries.hpp>
#include <arpirobot/devices/adafruitmotorhat/AdafruitMotorHatMotor.hpp>
#include <arpirobot/core/drive/ArcadeDriveHelper.hpp>
#include <arpirobot/devices/gamepad/Gamepad.hpp>
#include <arpirobot/devices/gamepad/ButtonPressedTrigger.hpp>
#include <arpirobot/core/drive/CubicAxisTransform.hpp>
#include <arpirobot/core/drive/SquareRootAxisTransform.hpp>
#include <arpirobot/arduino/iface/ArduinoUartInterface.hpp>
#include <arpirobot/arduino/sensor/Mpu6050Imu.hpp>


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

    // Custom function to control all brake mode settings at once
    void setBrakeMode(bool brakeMode);

    // Add devices and constants here as member objects
    // These should be public so actions can access them using Main::robot

    // Sensors and arduino interface
    Mpu6050Imu imu;
    ArduinoUartInterface arduino {"/dev/ttyUSB0", 57600};

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

    // Button numbers
    const int ROTATE_BTN = 0;

    // Joystick values beteween -deadband and +deadband are treated as zero
    // A joystick will generally not read exactly zero, so treat anything small 
    // enough as a zero to prevent trying to move the motors very slowly
    const double DEADBAND = 0.1;

    // Action instances
    JSDriveAction jsdriveAct;
    RotateAction rotateAct {90};

    // Action series
    ActionSeries rotateSer {{&rotateAct}, &jsdriveAct};

    // Button triggers to run actions on button presses
    ButtonPressedTrigger rotateTrigger {&gp0, ROTATE_BTN, &rotateSer};

    // Network table keys
    const std::string ROTATE_KP_KEY = "Rotate kP";
    const std::string ROTATE_KI_KEY = "Rotate kI";
    const std::string ROTATE_KD_KEY = "Rotate kD";
    const std::string IMU_Z_ANGLE_KEY = "Current Angle";
    const std::string ROTATE_TARGET_KEY = "Target Angle";
    const std::string ROTATE_ACTIVE_KEY = "Rotate Active";

    // PID used when rotating the robot (used by action)
    PID rotatePid {0.1, 0.0001, 0.01};
};