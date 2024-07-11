#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <windows.h>
#include "TeachMover.h"

using namespace std;

string formatCmd(int spd, int j1, int j2, int j3, int j4, int j5, int j6) {
    stringstream oss;
    oss << "@STEP " << to_string(spd) << ", " << to_string(j1) << ", " << to_string(j2) << ", " << to_string(j3) + ", " << to_string(j4) + ", " << to_string(j5) + ", " << to_string(j6);
    return oss.str();
}

void TeachMover::printLastErr() {
    DWORD error = GetLastError();
    LPVOID errorMsg;
    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        error,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR)&errorMsg,
        0, NULL);

    cerr << "Serial port error code " << error << ": " << (char*)errorMsg;
    LocalFree(errorMsg);
}

TeachMover::TeachMover(string portID) {
    // // Calculate the length of the wide string
    // int len = MultiByteToWideChar(CP_ACP, 0, portID.c_str(), -1, NULL, 0);
    // // Allocate memory for the wide string
    // wchar_t* portID_wstr = new wchar_t[len];
    // // Convert the string
    // MultiByteToWideChar(CP_ACP, 0, portID.c_str(), -1, portID_wstr, len);

    // std::wstring stemp = std::wstring(portID.begin(), portID.end());
    // LPCWSTR sw = stemp.c_str();

    // Opens the serial port
    hSerial = CreateFileA(portID.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        cerr << "Error opening serial port" << endl;
        return; // INVALID_HANDLE_VALUE;
    }
    printLastErr();

    // Configure the serial port
    dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error getting serial port state" << std::endl;
        CloseHandle(hSerial);
        return; // INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    cout << "updated hSerial: " << hSerial << endl;

    // Sets motor step attributes to starting position values
    m1 = 1768;
    m2 = 1100;
    m3 = 1040;
    m4 = 420;
    m5 = 0;
    m6 = 900;
    // self.lock = threading.Lock();
}

TeachMover::~TeachMover() {
    close_conn();
}

bool TeachMover::send_cmd(string cmd) {
    // cout << hSerial << endl;
    if (hSerial == INVALID_HANDLE_VALUE) {
        cerr << "Handle is invalid value in send_cmd" << endl;
    }

    if (!cmd.empty() && cmd.back() != '\r') {
        cmd += "\r";
    }

    /*
    # Attempt at threading ver
    def msg_robot(cmd):
        with self.lock:
            self.con.write(cmd.encode())
            return self.con.readline().decode().strip()

    if self.lock.locked():
            print("Currently locked")
            return "locked"

    thread = threading.Thread(target=msg_robot, args=[cmd])
    response = thread.start()
    */

    DWORD bytesWritten = 0;
    bool result = WriteFile(hSerial, cmd.c_str(), cmd.length(), &bytesWritten, NULL);
    if (result) {
        cout << "Data written to serial port: " << cmd << endl;

        // Wait for the transmission to complete - Doesn't work, probably bc of a robot issue
        // DWORD eventMask = 0;
        // SetCommMask(hSerial, EV_TXEMPTY);
        // WaitCommEvent(hSerial, &eventMask, NULL);
        this_thread::sleep_for(5s);
    }
    else {
        cerr << "Error writing to serial port" << endl;
        printLastErr();
    }
    return result;
}

bool TeachMover::move(int spd, int j1, int j2, int j3, int j4, int j5, int j6) {
    int gripper_adj = 0;
    if (j3 > 0) {
        gripper_adj = int(0.5 * j3);
    }
    else {
        gripper_adj = int(0.25 * j3);
    }
    string cmd = formatCmd(spd, j1, j2, j3, j4 + j5, j4 - j5, j6 + gripper_adj);
    bool response = this->send_cmd(cmd);
    cout << cmd << endl;
    this->increment_motors(j1, j2, j3, j4 + j5, j4 - j5, j6 + gripper_adj);
    return response;
}

bool TeachMover::set_step(int spd, int j1, int j2, int j3, int j4, int j5, int j6) {
    // prevent overextending the robot
    if (j1 > 7072 || j1 < 0 || j2>7072 || j2 < 0 || j3>4158 || j3 < 0 || j4>1536 || j4 < 0 || j5>1536 || j5 < 0) {
        cout << "Given motor step values are out of range" << endl;
        return false;
    }
    int diff1 = j1 - m1;
    int diff2 = j2 - m2;
    int diff3 = j3 - m3;
    int diff4 = j4 - m4;
    int diff5 = j5 - m5;
    int diff6 = j6 - m6;
    int gripper_adj = int(0.5 * diff3);
    // FIXME: diff2 causes the gripper to open and close
    string cmd = formatCmd(spd, diff1, diff2, diff3, diff4 + diff5, diff4 - diff5, diff6 + gripper_adj);
    bool response = this->send_cmd(cmd);
    // if response != "locked":
    cout << "Going to motor steps " << j1 << " " << j2 << " " << j3 << " " << j4 << " " << j5 << " " << j6 << endl;
    this->increment_motors(diff1, diff2, diff3, diff4 + diff5, diff4 - diff5, diff6 + gripper_adj);
    cout << cmd << endl;
    return response;
}

bool TeachMover::returnToStart() {
    /*
    currentPos = self.readPosition().split(',')
    j1 = -int(currentPos[0])
    j2 = -int(currentPos[1])
    j3 = -int(currentPos[2])
    j4 = -int(currentPos[3])
    j5 = -int(currentPos[4])
    j6 = -int(currentPos[5])-j3
    */
    int j1 = 1768 - m1;
    int j2 = 1100 - m2;
    int j3 = 1040 - m3;
    int j4 = 420 - m4;
    int j5 = -1 * m5;
    int j6 = 900 - m6;
    // ret = self.move(240, 1768-self.m1, 1100-self.m2, 1040-self.m3, -self.m4, -self.m5, 750-self.m6)
    string cmd = formatCmd(240, j1, j2, j3, j4, j5, j6);
    bool response = send_cmd(cmd);
    if (!response) {
        cout << "returning to zero position" << endl;
        cout << cmd << endl;
        increment_motors(j1, j2, j3, j4, j5, j6);
    }
    return response;
}

void TeachMover::increment_motors(int j1, int j2, int j3, int j4, int j5, int j6) {
    m1 += j1;
    m2 += j2;
    m3 += j3;
    m4 += j4;
    m5 += j5;
    m6 += j6;
    cout << "New updated ";
    print_motors();
}

void TeachMover::set_motor_vals(int m1, int m2, int m3, int m4, int m5, int m6) {
    this->m1 = m1;
    this->m2 = m2;
    this->m3 = m3;
    this->m4 = m4;
    this->m5 = m5;
    this->m6 = m6;
}

void TeachMover::print_motors() {
    cout << "step motors: " << m1 << ", " << m2 << ", " << m3 << ", " << m4 << ", " << m5 << ", " << m6 << endl;
}

// Resets what the robot considers its "0 Position"
void TeachMover::reset() {
    cout << "Resetting 0 position" << endl;
    m1, m2, m3, m4, m5, m6 = 0, 0, 0, 0, 0, 0;
    send_cmd("@RESET");
}

// string TeachMover::read_pos() {
//     cout << "Reading position" << endl;
//     string cmd = "@READ\r";
//     this->con.flushInput();
//     this->con.write(cmd.encode());
//     time.sleep(1);
//     string response = self.con.readline().decode().strip();
//     // Strip the leading status code
//     return response;
// }

// string TeachMover::readPosition() {
//     string ret = send_cmd("@READ\r");
//     // Strip the leading status code;
//     ret = ret.split('\r')[-1];
//     return ret;
// }

void TeachMover::close_grip() {
    cout << "Closing grip" << endl;
    // # TODO: UPDATE ROBOT MOTOR VALUES
    // # @CLOSE IS VERY SLOW SO I'M USING A MOVE INSTEAD
    // # self.send_cmd("@CLOSE")
    // # self.set_motor_vals(self.m1, self.m2, self.m3, self.m4, self.m5, 0)
    move(240, 0, 0, 0, 0, 0, -1 * m6);
}

void TeachMover::open_grip() {
    if (m6 < 1400) {
        cout << "Opening grip" << endl;
        move(240, 0, 0, 0, 0, 0, 1500 - m6);
    }
    else {
        cout << "Grip already open" << endl;
    }
}

void TeachMover::lock_wait() {
    // while (this->lock.locked()) {
    //     continue;
    // }
    // return;
}

void TeachMover::close_conn() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        std::cout << "Serial port closed" << std::endl;
    }
}

// int counter = 0;
// void TeachMover::test_thread(int num) {
//     void msg_robot(){
//         if (self.lock.locked()) {
//             print(f"Currently locked for msg {num}");
//             return;
//         }
//         with (self.lock) {
//             print(f"starting msg {num}");
//             self.counter += 1;
//             time.sleep(0.2);
//             print(f"finish msg {num}");
//         }
//     }

//     thread = threading.Thread(target=msg_robot);
//     thread.start();
//     // print(response)
//     return
// }       


int main(int argc, char* argv[]) {
    TeachMover robot("COM3");
    cout << "----- done initializing ---------" << endl;
    // cout << robot.hSerial << endl;
    // robot.move(240, 400, 400, 0, 0, 0, 0);
    robot.set_step(240, 1768, 3100, 1440, 420, 0, 900);
    robot.returnToStart();

    //     # # INVERSE KINEMATICS TESTING
    //     # IK = InverseKinematics()
    //     # j1, j2, j3, j4, j5 = IK.FindStep(7, 0, 0, 0, 0)
    //     # print(f"results: {j1}, {j2}, {j3}, {j4}, {j5}")
    //     # robot.set_step(240, j1, j2, j3, j4, j5, robot.m6)

    //     # @READ TESTS
    //     # response = robot.read_pos()
    //     # print(response)
    //     # response2 = robot.readPosition()
    //     # print(response2)

    //     # NOTE: Default position is (1768, 1100, 1040, 420, 0, 900)
    //     # robot.move(240, 0,1200,800,0,0,0)
    //     robot.set_step(240, 1768, 3100, 1440, 420, 0, 900)
    //     # robot.lock_wait()
    //     robot.returnToStart()

    //     # NOTE: BASIC THREAD TESTING
    //     # robot.test_thread(1)
    //     # for i in range(10):
    //     #     print(f"Iteration {i}")
    //     #     robot.test_thread(i)
    //     #     time.sleep(0.1)
    //     # time.sleep(5)
    //     # print(robot.counter)

    //     # NOTE: ROBOT MOVE THREAD TESTING
    //     # i = 0
    //     # while True:
    //     #     print(f"iteration {i}")
    //     #     robot.move(200,100,10,0,0,0,0)
    //     #     time.sleep(0.1)
    //     #     i += 1

    //     # NOTE: @READ TESTING
    //     # pos = robot.readPosition()
    //     # print(type(pos))
    //     # print(pos)
}