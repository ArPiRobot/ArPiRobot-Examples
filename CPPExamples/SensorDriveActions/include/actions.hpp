#pragma once

#include <arpirobot/core/action/Action.hpp>
#include <chrono>

using namespace arpirobot;

/**
 * Action to drive the robot using gamepad input
 * This action will never terminate on its own. It will only terminate if 
 * interrupted either by using ActionManager::stopAction or if another action is started that takes
 * control of a device that this action used
 */
class JSDriveAction : public Action {
protected:
    // Run when the action is started. 
    // If an action needs exclusive control of devices, lock them here
    void begin() override;

    // Run periodically while the action is running
    void process() override;

    // Run when the action is stopped
    // wasInterrupted will be true if the action did not stop on its own (see shouldContinue)
    void finish(bool wasInterrupted) override;

    // The action will stop itself when this returns false
    // This is run after each time process() runs to determine if the action should stop
    bool shouldContinue() override;
};

/**
 * Action to drive at a certain speed for a certain number of encoder ticks
 * Also includes a timeout
 */
class DriveDistanceAction : public Action {
public:
    // Custom constructor allows configuring the action when instantiating
    DriveDistanceAction(double driveSpeed, int encoderTicks, int timeoutMs);

protected:
    // Run when the action is started. 
    // If an action needs exclusive control of devices, lock them here
    void begin() override;

    // Run periodically while the action is running
    void process() override;

    // Run when the action is stopped
    // wasInterrupted will be true if the action did not stop on its own (see shouldContinue)
    void finish(bool wasInterrupted) override;

    // The action will stop itself when this returns false
    // This is run after each time process() runs to determine if the action should stop
    bool shouldContinue() override;

private:
    // Store values passed to constructor
    double driveSpeed;
    int encoderTicks, timeoutMs;

    // Track when this action started
    std::chrono::time_point<std::chrono::steady_clock> startTime;
};

/**
 * Action to wait some amount of time
 */
class WaitTimeAction : public Action {
public:
    WaitTimeAction(int timeMs);

protected:
    // Run when the action is started. 
    // If an action needs exclusive control of devices, lock them here
    void begin() override;

    // Run periodically while the action is running
    void process() override;

    // Run when the action is stopped
    // wasInterrupted will be true if the action did not stop on its own (see shouldContinue)
    void finish(bool wasInterrupted) override;

    // The action will stop itself when this returns false
    // This is run after each time process() runs to determine if the action should stop
    bool shouldContinue() override;

private:
    int timeMs;
    std::chrono::time_point<std::chrono::steady_clock> startTime;
};


/**
 * Action to rotate a certain number of degrees based on the robot's IMU
 * This action will rotate the given number of degrees from the starting angle
 */
class RotateDegreesAction : public Action{
public:
    RotateDegreesAction(double rotationSpeed, double degrees, int timeoutMs);

protected:
    // Run when the action is started. 
    // If an action needs exclusive control of devices, lock them here
    void begin() override;

    // Run periodically while the action is running
    void process() override;

    // Run when the action is stopped
    // wasInterrupted will be true if the action did not stop on its own (see shouldContinue)
    void finish(bool wasInterrupted) override;

    // The action will stop itself when this returns false
    // This is run after each time process() runs to determine if the action should stop
    bool shouldContinue() override;

private:
    std::chrono::time_point<std::chrono::steady_clock> startTime;
    double startDegrees;

    double rotationSpeed, degrees;
    int timeoutMs;
};
