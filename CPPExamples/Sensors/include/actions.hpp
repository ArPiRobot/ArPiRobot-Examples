#pragma once

#include <arpirobot/core/action/Action.hpp>

using namespace arpirobot;

// Define actions here
// Actions must have 4 functions. Use the template below

/*class MyAction : public Action{
public:
    // You can define a custom constructor here (with arguments) if needed

protected:
    // Run when the action is started. 
    // If an action needs exclusive control of devices, lock them here
    void begin();

    // Run periodically while the action is running
    void process();

    // Run when the action is stopped
    // wasInterrupted will be true if the action did not stop on its own (see shouldContinue)
    void finish(bool wasInterrupted);

    // The action will stop itself when this returns false
    // This is run after each time process() runs to determine if the action should stop
    bool shouldContinue();

private:
    // Member variables (usually you just need any arguments passed to a custom constructor)
};*/