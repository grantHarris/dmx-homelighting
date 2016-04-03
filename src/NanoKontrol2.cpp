#include <iostream>
#include "RtMidi.h"
#include "NanoKontrol2.h"

NanoKontrol2::NanoKontrol2(unsigned int port): buffer(72){
  RtMidiIn *midi_in = new RtMidiIn();
  if (midi_in->getPortCount() == 0) {
    cout << "No MIDI ports available\n"<<endl;;
  }else{
    buffer[0] = 0.8;

    midi_in->openPort(port);
    
    midi_in->setCallback(&readNanoKontrol, static_cast<void*>(&buffer));
    //midi_in->setCallback(&readNanoKontrol, &bar);
    midi_in->ignoreTypes(false, false, false);
  }
}

NanoKontrol2::~NanoKontrol2(){
}

float NanoKontrol2::getValue(unsigned int address){
  return buffer[address];
}

void NanoKontrol2::readNanoKontrol(double delta, std::vector<unsigned char> *message, void *userData){

  /*
   * MIDI Mappings
   *
   * Sliders  0 - 7
   * Knobs  16 - 23
   * S Button 32 - 39

   * Play    41
   * Stop   42
   * FF   43
   * RV   44
   * REC   45
   * Cycle  46

   * M Button 48 - 55

   * Track Back  58
   * Track FWD  59

   * Marker Set 60
   * Marker Back  61
   * Marker FWD 62
   * R Button 64 - 71
   */

  vector <float> *buffer = static_cast <vector <float> *> (userData);
  unsigned int number_bytes = message->size();
  
  unsigned int address = message->at(1);
  float value = (float) message->at(2) / 127.0;
  buffer->at(address) = value;

}


