
// #define __DBG_FOR_DUMMIES__
// #define __HELP_COMMENTS__
//#define __INSTANT_STIMULATION__
// #define __SIGNAL_STIM_ON_LED__
#define __RETURN_PULSE_COMPLETION__

#include "pulse_def.hpp"
#include "communication.hpp"

// Global class instances
StimulatingProbe* probe;

// Pin configuration
// Make sure that you are not using the TX, RX pins as we are using serial communication explicitly 
const int probe_cathode = 6;
const int probe_anode   = 9;
const int signal_LED    = 13;
const int signal_DELAY  = 50;

const int STIM_BUFFER_SIZE = 10;
unsigned long stim_q[STIM_BUFFER_SIZE];
const unsigned long STIM_CTRL_DELAY = 200;
int stim_q_front = 0;
int stim_q_back = 0;

const unsigned long ARDUINO_REFRAC = 50;
unsigned long old_stim_m = 0;

// Initial Setup
void setup() { 
  initCOM();

  // Initialize the stimulating probe struct
  probe = initProbe(probe_cathode, probe_anode);

  #ifdef __SIGNAL_STIM_ON_LED__
    pinMode(signal_LED, OUTPUT);
    digitalWrite(signal_LED, LOW);
  #endif
#ifdef __HELP_COMMENTS__
  Serial.println("Available commands: [m]ove/set [d]irection/[t]oggle state/[s]top/[p]rint status");
#endif

#ifndef __INSTANT_STIMULATION__
  stim_q_front = 0;
  stim_q_back = 0;
#endif
}

// Local variables not repeatedly initialized
char com_input;

// Local functions
void processCOMInput(char &in) {
  if (in == DEFAULT_COM_OUTPUT) {
    return;
  }
}

void loop() {
  // Read any inputs from the Serial Monitor
  com_input = readCOM();
  #ifdef __INSTANT_STIMULATION__
    if (com_input == TRIGGER_STIM) {
      sendPulse(probe);
      #ifdef __SIGNAL_STIM_ON_LED__
        digitalWrite(signal_LED, HIGH);
        delay(signal_DELAY);
        digitalWrite(signal_LED, LOW);
      #endif
      // processCOMInput(com_input);
      #ifdef __RETURN_PULSE_COMPLETION__
        // Serial.println("SC");
      #endif
    }
  #else
    // Put the stimulation time in an array
    if (com_input == TRIGGER_STIM) {
      if (millis() - old_stim_m > ARDUINO_REFRAC) {
        // Insert stimulation time into an array
        stim_q[stim_q_front] = millis() + STIM_CTRL_DELAY;
        stim_q_front = (stim_q_front+1) % STIM_BUFFER_SIZE;
  
        //if buffer is full, just drop the least recent one
        //of course, buffer is big enough this should never happen!
        if (stim_q_front == stim_q_back) {
          stim_q_back = (stim_q_back+1) % STIM_BUFFER_SIZE;
        }
      }
    }
    if (stim_q_front != stim_q_back &&
        millis() >= stim_q[stim_q_back]) {
      sendPulse(probe);
      old_stim_m = millis();
      stim_q_back = (stim_q_back+1) % STIM_BUFFER_SIZE;
      #ifdef __SIGNAL_STIM_ON_LED__
        digitalWrite(signal_LED, HIGH);
        delay(signal_DELAY);
        digitalWrite(signal_LED, LOW);
      #endif
    }
  #endif
}
