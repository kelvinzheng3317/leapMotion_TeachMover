#pragma once
#include <iostream>
#include <string>
#include <mutex>

using namespace std;

class Buffer {
    mutex mtx;
    string data;

    public:
        Buffer();
        void add(string msg);
        string get();
        bool isEmpty();
};