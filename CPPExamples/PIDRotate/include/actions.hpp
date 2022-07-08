#pragma once

#include <arpirobot/core/action/Action.hpp>

using namespace arpirobot;

/**
 * Action to drive the robot using gamepad input
 * This action will never terminate on its own. It will only terminate if 
 * interrupted either by using ActionManager::stopAction or if another action is started that takes
 * control of a device that this action used
 */
class JSDriveAction : public Action {
protected:
    // If an action needs exclusive control of devices, lock them here
    LockedDeviceList lockedDevices() override;

    // Run when the action is started. 
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
 * Action to rotate the robot a specified number of degrees using the IMU
 * This action will terminate once rotation is done
 */
class RotateAction : public Action {
public:
    RotateAction(double degrees);

protected:
    // If an action needs exclusive control of devices, lock them here
    LockedDeviceList lockedDevices() override;

    // Run when the action is started. 
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
    int correctCounter;
    double degrees;
};
