#include <Balance.h>
// Micah 11-7-17
  int Value = 0;
void setup() {
}

  Balance control(1,1,1,0);
void loop() {
  control.UpdatePID(Value);
}

// This is a fork edit
// Hello this is a test branch!
