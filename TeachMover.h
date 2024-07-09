#pragma once
#include <string>
#include <windows.h>

using namespace std;

class TeachMover {
    int m1 = 1768;
    int m2 = 1100;
    int m3 = 1040;
    int m4 = 420;
    int m5 = 0;
    int m6 = 900;

    DCB dcbSerialParams;
    HANDLE hSerial;

    public:
        TeachMover(string portID);
        ~TeachMover();

        bool send_cmd(string cmd);

        // Moves the robot based a set amount from current position
        bool move(int spd, int j1, int j2, int j3, int j4, int j5, int j6);

        // Moves the robot to fixed motor stepper values
        bool set_step(int spd, int j1, int j2, int j3, int j4, int j5, int j6);

        bool returnToStart();
        string read_pos();
        string readPosition();

        void increment_motors(int j1, int j2, int j3, int j4, int j5, int j6);
        void set_motor_vals(int m1, int m2, int m3, int m4, int m5, int m6);
        void print_motors();
        void reset();
        void close_grip();
        void open_grip();
        void lock_wait();
        void printLastErr();
        void close_conn();


};