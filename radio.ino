#include <RF24.h> 
#include <Wire.h>
#include <ssd1306.h> // https://github.com/lexus2k/ssd1306

#define MCP4725_ADDR 0x60

char msg[32];
RF24 radio(9,10);
const uint64_t pipe = 0xF0F0F0F0AA;

int s2;
int ls2 = HIGH;
unsigned long ld2 = 0;
unsigned long debounceDelay = 50;

void setup(void) {
  Serial.begin(9600);
  Wire.begin();

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  
  if (radio.begin()) {
    Serial.println("radio began");
    radio.openReadingPipe(1, pipe);
    radio.openWritingPipe(pipe);
    
    radio.startListening();
  } else {
    Serial.println("radio began");
  }

  ssd1306_128x64_i2c_init();
  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_clearScreen();
  ssd1306_printFixed(0,  0, "listening...", STYLE_NORMAL);
}

void loop(void) {
  int d2 = digitalRead(2);

  if (d2 != ls2) ld2 = millis();

  if ((millis() - ld2) > debounceDelay) {
    if (d2 != s2) {
      s2 = d2;

      if (s2 == LOW) {
        ssd1306_clearScreen();
        ssd1306_printFixed(0,  0, "transmitting...", STYLE_NORMAL);
        
        radio.stopListening();

        char t[] = "hello from device #1";
        radio.write(t, sizeof(t));

        delay(1000);
      
        radio.startListening();
        ssd1306_clearScreen();
        ssd1306_printFixed(0,  0, "listening...", STYLE_NORMAL);
      }
    }
  }
  
  if (radio.available()) {
    radio.read(&msg, sizeof(msg));

    ssd1306_clearScreen();
    ssd1306_printFixed(0,  0, msg, STYLE_NORMAL);
    delay(5000);
    ssd1306_clearScreen();
    ssd1306_printFixed(0,  0, "listening...", STYLE_NORMAL);
  }

  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                          // cmd to update the DAC
  Wire.write(0);        // the 8 most significant bits...
  Wire.endTransmission();

  ls2 = d2;
}
