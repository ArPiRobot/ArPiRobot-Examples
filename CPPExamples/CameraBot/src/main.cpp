#include <main.hpp>

Robot *Main::robot = nullptr;

int main(){
    Main::robot = new Robot();
    Main::robot->start();
}