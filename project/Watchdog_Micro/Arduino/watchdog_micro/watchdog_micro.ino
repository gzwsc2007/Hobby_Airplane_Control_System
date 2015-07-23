#include <avr/io.h>

#define RC_CHAN_WIDTH_MIDSCALE_US  (1500UL) // 1.5ms

#define RC_SEL_PASSTHROUGH         (LOW)
#define RC_SEL_NUCLEO              (HIGH)

#define ST_TO_TINY_CHECKIN_TIMEOUT_MS  (50) // 50ms

int rcSelPin = 4;
int rcRecvChan = 0;
int stToTinyCheckin = 3;
int debugPin = 1;

unsigned long width;
unsigned long last;
int stCheckinPinStatus;

void setup() {
  pinMode(debugPin, OUTPUT);
  pinMode(rcSelPin, OUTPUT);
  digitalWrite(rcSelPin, RC_SEL_PASSTHROUGH);
  pinMode(rcRecvChan, INPUT);
  pinMode(stToTinyCheckin, INPUT_PULLUP);
  
  stCheckinPinStatus = digitalRead(stToTinyCheckin);
}

void loop() {
  int newCheckinStatus = digitalRead(stToTinyCheckin);
  
  // Check if the ST checkin pin has changed
  if (stCheckinPinStatus != newCheckinStatus) {
    last = millis();
    stCheckinPinStatus = newCheckinStatus;
  }
 
  // Timeout for ST_to_tiny_checkin
  if (millis() - last > ST_TO_TINY_CHECKIN_TIMEOUT_MS) {
    // Override RC chan 6. Force to use passthrough mode.
    digitalWrite(rcSelPin, RC_SEL_PASSTHROUGH);
    return;
  }

  // Read receiver_chan_6
  width = pulseIn(rcRecvChan, HIGH, 50000UL); // read RC chan 6, timeout 50ms
  if (width != 0) {
    if (width >= RC_CHAN_WIDTH_MIDSCALE_US) {
      digitalWrite(rcSelPin, RC_SEL_NUCLEO);
    } else {
      digitalWrite(rcSelPin, RC_SEL_PASSTHROUGH);
    }
  }
}
