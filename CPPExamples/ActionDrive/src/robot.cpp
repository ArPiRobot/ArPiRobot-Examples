#include <robot.hpp>

#include <arpirobot/core/log/Logger.hpp>
#include <arpirobot/core/action/ActionManager.hpp>
#include <arpirobot/core/network/NetworkTable.hpp>

using namespace arpirobot;


void Robot::robotStarted(){
    // Setup axis transforms
    gp0.setAxisTransform(DRIVE_AXIS, std::make_shared<CubicAxisTransform>(0, 0.5));
    gp0.setAxisTransform(ROTATE_AXIS, std::make_shared<SquareRootAxisTransform>());

    // Fix motor directions (as needed, depends on wiring)
    flmotor.setInverted(true);
    frmotor.setInverted(true);

    // Start this action when the robot starts
    // It will keep running unless interrupted
    ActionManager::startAction(std::make_shared<JSDriveAction>());
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
    // Nothing here. Driving handled by JSDriveAction
}

void Robot::disabledPeriodic(){

}

void Robot::periodic(){
    // Do not remove this line or some devices will be disabled.
    feedWatchdog();
}