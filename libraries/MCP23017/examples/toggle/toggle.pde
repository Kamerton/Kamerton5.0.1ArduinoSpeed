#include <Wire.h>
#include "MCP23017.h"

// Basic pin reading and pullup test for the MCP23017 I/O expander
// public domain!

// Connect pin #12 of the expander to Analog 5 (i2c clock)
// Connect pin #13 of the expander to Analog 4 (i2c data)
// Connect pins #15, 16 and 17 of the expander to ground (address selection)
// Connect pin #9 of the expander to 5V (power)
// Connect pin #10 of the expander to ground (common ground)

// Output #0 is on pin 21 so connect an LED or whatever from that to ground

MCP23017 mcp;
  
void setup() {  
  mcp.begin(5);      // use default address 0

  mcp.pinMode(15, OUTPUT);
}


// flip the pin #0 up and down

void loop() {
  delay(100);

  mcp.digitalWrite(15, HIGH);

  delay(100);

  mcp.digitalWrite(15, LOW);
}