#include <iostream>
#include <string>
#include <mutex>
#include "buffer.h"

using namespace std;

Buffer::Buffer() {
    data = "";
}

void Buffer::add(string msg) {
    mtx.lock();
    data = msg;
    mtx.unlock();
}

string Buffer::get() {
    mtx.lock();
    string msg = data;
    data = "";
    mtx.unlock();
    return msg;
}

bool Buffer::isEmpty() {
    // cout << "debugging isEmpty data: " << data << ", length: " << data.length() << endl;
    return data.length() == 0;
}


// int main(int argc, char* argv[]) {
//     Buffer buffer;
//     cout << buffer.get() << endl;

//     buffer.add("V pos");
//     buffer.add("Fist");
//     buffer.add("1, 2, 3");
//     string item = buffer.get();
//     cout << item << endl;
// }