#pragma once

#include <arpirobot/core/robot/BaseRobot.hpp>

#include <actions.hpp>

#include <arpirobot/arduino/iface/ArduinoUartInterface.hpp>
#include <arpirobot/arduino/sensor/VoltageMonitor.hpp>
#include <arpirobot/arduino/sensor/SingleEncoder.hpp>
#include <arpirobot/arduino/sensor/Mpu6050Imu.hpp>
#include <arpirobot/arduino/sensor/Ultrasonic4Pin.hpp>
#include <arpirobot/arduino/sensor/IRReflectorModule.hpp>


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

    // Object used to communicate with an arduino connected to the raspberry pi via UART (serial)
    // First argument is the serial port name. Typically this will either be /dev/ttyUSB0 or /dev/tty/ACM0
    // Occasionally, the number may be different, but usually only if multiple arduinos are connected
    // The second argument is the buad rate. This needs to match the value set in the 
    // arduino firmware flashed to the arduino. By default, this is 57600
    ArduinoUartInterface arduino {"/dev/ttyUSB0", 57600};

    // Simple voltage divider used to measure battery voltage.
    // This is used for the cheap voltage "sensors" that can be found online
    // First argument is the pin name. This depends on how you connected it to the arduino
    // Second argument is the voltage of the arduino board (either 5V or 3.3V usually)
    // Third argument is the resistance from power to measurement point (R1)
    // Fourth is the resistance from gnd to the measurement point (R2)
    // These resistances depend on how the voltage divider is constructed
    // For the cheap "sensors" purchased online these are R1 = 30000, R2 = 7500
    VoltageMonitor vmon {"A0", 5.00, 30000, 7500};

    // SingleEncoder is used with a single channel encoder.
    // The first argument is the pin the encoder is connected to (depends on wiring to arduino)
    // The second argument indicates if an internall pullup resistor should be used.
    // Generally, this should be false if using a prebuilt module
    SingleEncoder lencoder {2, false};
    SingleEncoder rencoder {3, false};

    // This is one type of IMU that can be used with the arduino
    Mpu6050Imu imu;

    // Ultrasonic sensor with 4 pins (2 power, 2 I/O)
    // The two I/O pins are echo and trigger.
    // First argument is trigger pin number, second is echo pin number. Depends on wiring
    Ultrasonic4Pin usonic {7, 8};

    // IR reflector modules
    // Only argument used here is the pin the digital output of the module is connected to
    // Some modules also have analog outputs, which can optionally be specified as the second argument
    IRReflectorModule ldetector {11};
    IRReflectorModule rdetector {12};
};
