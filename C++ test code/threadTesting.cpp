#include <iostream>
#include <pthread.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <sstream>
#include "buffer.h"

using namespace std;

atomic<bool> stop_thread(false);

vector<float> stringToVec(string str) {
    vector<float> coords;
    stringstream ss(str);
    string int_str;
    try {
        for (int i=0; i<3; ++i) {
            getline(ss, int_str, ' ');
            coords.push_back(stof(int_str));
        }
    } catch (...) {
        cout << "Error: data couldn't be converted to coordinates" << endl;
        coords = {};
    }
    return coords;
}

// This is the pthreads version of threadFunc
void* pthreadFunc(void* arg) {
    string* msg = static_cast<string*>(arg);
    for (int i=0; i<3; ++i) {
        cout << *msg << endl;
        this_thread::sleep_for(1s);
    }
    cout << "Exiting thread" << endl;
    return nullptr;
}

void threadFunc(Buffer* buffer) {
    while(!stop_thread) {
        // cout << "thread loop iteration" << endl;
        if (!buffer->isEmpty()) {
            vector<float> coords = stringToVec(buffer->get());
            if (coords.size() == 3) {
                cout << "coords: " << coords[0] << ", " << coords[1] << ", " << coords[2] << endl;
            }
        } else {
            cout << "thread empty" << endl;
        }
        this_thread::sleep_for(0.5s);
    }
    cout << "Exiting thread" << endl;
}

int main(int argc, char* argv[]) {
    string data = "test";
    Buffer* buffer = new Buffer();

    // pthread_t thread1;
    // if (pthread_create(&thread1, nullptr, pthreadFunc, &data)) {
    //     std::cerr << "Error creating thread" << std::endl;
    //     return 1;
    // }
    // if (pthread_join(thread1, nullptr)) {
    //     std::cerr << "Error joining thread" << std::endl;
    //     return 2;
    // }

    // thread t1(threadFunc, buffer);
    // this_thread::sleep_for(1s);
    // for (int i=0; i<5; ++i) {
    //     buffer->add(to_string(i) + " " + to_string(i) + " " + to_string(i));
    //     cout << "main thread iteration " << i << endl;
    //     this_thread::sleep_for(1s);
    // }
    // stop_thread = true;
    // t1.join();

    string str = to_string(0.313131313123515) + " " + to_string(0.313131313123515) + " " + to_string(0.313131313123515);
    cout << str << endl;
    delete buffer;

    cout << "Program finishing" << endl;
}