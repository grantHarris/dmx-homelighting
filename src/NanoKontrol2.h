#ifndef NANO_KONTROL_2_H
#define NANO_KONTROL_2_H
#include <iostream>
#include <vector>
#include "RtMidi.h"

using std::vector;
using namespace std;

class NanoKontrol2{
  public:
    NanoKontrol2(unsigned int port);
    ~NanoKontrol2();
    float getValue(unsigned int address);
 private:
    static void readNanoKontrol(double delta, std::vector<unsigned char> *message, void *userData);
    vector<float> buffer;
};


#endif
