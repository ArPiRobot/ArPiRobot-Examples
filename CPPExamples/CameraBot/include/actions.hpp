#pragma once

#include <arpirobot/action/Action.hpp>

using namespace arpirobot;

// Define actions here
// Actions must have 4 functions. Use the template below
// An optional 5th function (lockedDevices) is 
// available if an action needs exclusive control of a device

/*
class MyAction : public Action{
public:
    // Add arguments to constructor if needed
    MyAction();

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
    // Member variables (usually you just need any arguments passed to a custom constructor)
};
*/