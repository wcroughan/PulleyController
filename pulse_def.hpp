#ifndef __PWM_H_INCLUDED__
#define __PWM_H_INCLUDED__

#define STIM_PULSE_WIDTH 200  // Pulse width in us
#define STIM_PULSE_DELAY 200  // Delay in ms
#define STIM_NOP __asm__ __volatile__ ("nop\n\t")

struct StimulatingProbe {
  int p_ON;
  int p_OFF;
};


StimulatingProbe* initProbe(const int _p_on, const int _p_off) {
  StimulatingProbe* stim_probe = new StimulatingProbe;
  stim_probe->p_ON  = _p_on;
  stim_probe->p_OFF = _p_off;  

  // Set an initial pin voltages - Set everything to ground.
  pinMode(stim_probe->p_ON, OUTPUT);
  digitalWrite(stim_probe->p_ON, LOW);

  pinMode(stim_probe->p_OFF, OUTPUT);
  digitalWrite(stim_probe->p_OFF, LOW);

  return stim_probe;
}


//void sendPulse(StimulatingProbe* stim_probe, unsigned long stim_time) {
// /* Function for genrating biphasic pulse on pin pair (pin_ON, pin_OFF)
//  *  Inputs:
//  *  stim_probe: Instance of the StimulatingProbe struct
//  *  stim_time: Time at which stimulation should happen
//  */
//
//  while (millis() < stim_time + STIM_PULSE_DLAY) {
//    STIM_NOP;
//  }
//  digitalWrite(stim_probe->p_OFF, HIGH);
//  delayMicroseconds(STIM_PULSE_WIDTH);
//  digitalWrite(stim_probe->p_OFF, LOW);
//  delayMicroseconds(20);
//  digitalWrite(stim_probe->p_OFF, HIGH);
//  delayMicroseconds(STIM_PULSE_WIDTH);
//  digitalWrite(stim_probe->p_OFF, LOW);
//}

void sendPulse(StimulatingProbe* stim_probe) {
 /* Function for genrating biphasic pulse on pin pair (pin_ON, pin_OFF)
  *  Inputs:
  *  stim_probe: Instance of the StimulatingProbe struct
  */

  // It is assumed that the PIN has been declared as an output beforehand
  /*
   * Works for MicroProbes
  digitalWrite(stim_probe->p_ON, HIGH);
  delayMicroseconds(STIM_PULSE_WIDTH);
  digitalWrite(stim_probe->p_ON, LOW);
  digitalWrite(stim_probe->p_OFF, HIGH);
  delayMicroseconds(STIM_PULSE_WIDTH);
  digitalWrite(stim_probe->p_OFF, LOW);
  */
  digitalWrite(stim_probe->p_OFF, HIGH);
  delayMicroseconds(STIM_PULSE_WIDTH);
  digitalWrite(stim_probe->p_OFF, LOW);
  delayMicroseconds(20);
  digitalWrite(stim_probe->p_OFF, HIGH);
  delayMicroseconds(STIM_PULSE_WIDTH);
  digitalWrite(stim_probe->p_OFF, LOW);
}

//void sendDelayedPulse(StimulatingProbe* stim_probe) {
// /* Function for genrating Pulse Width Modulation
//  *  Inputs:
//  *  stim_probe: Instance of the stimulating probe struct
//  */
//
//  delay(STIM_PULSE_DELAY);
//  sendPulse(stim_probe);
//}

#endif
