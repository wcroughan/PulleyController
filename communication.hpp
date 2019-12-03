#ifndef __COMMUNICATION_H_INCLUDED__
#define __COMMUNICATION_H_INCLUDED__
/*  Functions for commnuicating with a computer, taking inputs, and debugging
 *  
 *  Date: September 19, 2019
 */

// Definitions for constants used in the header file

#define BAUD_RATE 115200
#define SERIAL_BANNER "---- Starting Serial Monitor ----"
#define DEFAULT_COM_OUTPUT 0
#define TRIGGER_STIM 'T'
#define TOGGLE_ENABLE 't'
#define DIRECTION_FORWARD 'f'
#define DIRECTION_BACKWARD 'b'
#define PRINT_STATUS 'p'
#define KEYWORD_STOP 's'
#define NOP __asm__ __volatile__ ("nop\n\t")

// String buffer for reading non-character entries
String str_buffer;

// For reading integers, we have bounds on what values can be read.
int MIN_INT_TO_READ = 1; 
int MAX_INT_TO_READ = 1000;

// Initialize communication using Serial Port
// Make sure that TX and RX Pins (Pins 1 and 0) are not being used for any 
// other purpose on the board.
void initCOM() {
  Serial.begin(BAUD_RATE);
  Serial.println(SERIAL_BANNER);
}

int readInt() {
  // Read integer from Serial monitor
  bool read_data = false;
  int val        = 0;
  while (true) {
    if (Serial.available()) {
      val = Serial.parseInt();
      read_data = true;
    }

  #ifdef __DBG_FOR_DUMMIES__
    if (read_data) {
      Serial.print("Read input: ");
      Serial.println(val);
    }
  #endif
  
    // TODO: If different bounds are needed for different calls, they
    // can be made arguments to this function. Don't see any need for that
    // right now!
    if ((val >= MIN_INT_TO_READ) && (val <= MAX_INT_TO_READ)) {
      return val;
    }

    if (read_data) {
      Serial.print(val);
      Serial.print(" not in the specified bounds: (");
      Serial.print(MIN_INT_TO_READ);
      Serial.print(", ");
      Serial.print(MAX_INT_TO_READ);
      Serial.println(")");
      return DEFAULT_COM_OUTPUT;
    }
  }
}

void updateDirection(bool &curr_dir) {
#ifdef __HELP_COMMENTS__
  Serial.println("Set the current direction to [f]orward/[b]ackward");
#endif

  while (true) {
    if (Serial.available()) {
      char new_dir = Serial.read();
      if (new_dir == DIRECTION_FORWARD) {
        curr_dir = true;
        return;
      }
      
      if (new_dir == DIRECTION_BACKWARD) {
        curr_dir = false;
        return;
      }
  
      Serial.println("Invalid direction supplied, Aborting!");
    }
  }
}
char readChar() {
// Reach char from Serial monitor
  return DEFAULT_COM_OUTPUT;
}

char readCOM() {
  if (Serial.available()) {
    char c_in = Serial.read();
    switch(c_in) {
      case KEYWORD_STOP:
      case PRINT_STATUS:
      case TOGGLE_ENABLE:
      case TRIGGER_STIM:
        return c_in;
      default:
        return DEFAULT_COM_OUTPUT;
    }
  }

  // TODO: Insert NOPs if consistent timing is really needed!
  // Variablility in reading from the Serial Port can probably not be accounted for.
  return DEFAULT_COM_OUTPUT;
}

void directionalityTest(bool &retry, bool &outcome) {
  char answer;
  retry = false;
  Serial.println("Did the pulse have correct polarity? [y]es/[n]o/[r]etry");
  while (true) {
    if (Serial.available()) {
      answer = Serial.read();
      if (answer == 'y') {
        outcome = true;
        return;
      }
      
      if (answer == 'n') {
        outcome = false;
        return;
      }
      
      if (answer == 'r') {
        retry = true;
        return;
      }
      Serial.println("Did the motor move forward? [y]es/[n]o/[r]etry");
    }
  }
}
#endif
