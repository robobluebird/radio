#include <SPI.h>

#define WRITE_ENABLE 6
#define WRITE 2
#define READ 3
#define PAGE_SIZE 64
#define RECORD_LIMIT 80000

byte s1, s1_trigger, s1_latch;
byte ls1 = HIGH;
unsigned long ld1 = 0;

byte s2, s2_trigger, s2_latch;
byte ls2 = HIGH;
unsigned long ld2 = 0;

byte s3, s3_trigger, s3_latch;
byte ls3 = HIGH;
unsigned long ld3 = 0;

byte s4, s4_trigger, s4_latch;
byte ls4 = HIGH;
unsigned long ld4 = 0;

byte s5, s5_trigger, s5_latch;
byte ls5 = HIGH;
unsigned long ld5 = 0;

byte s6, s6_trigger, s6_latch;
byte ls6 = HIGH;
unsigned long ld6 = 0;

byte s7;
byte ls7 = HIGH;
unsigned long ld7 = 0;

byte continuous;

const unsigned long debounceDelay = 50;

const unsigned long dds_tune = 4294967296 / 8000;

int sample_out_temp;
byte sample_out;

byte pitch = 56;
byte minor_second = round(pitch * 1.059463);
byte major_second = round(pitch * 1.122462);
byte minor_third = round(pitch * 1.189207);
byte major_third = round(pitch * 1.259921);
byte perfect_fourth = round(pitch * 1.334840);
byte tritone = round(pitch * 1.414214);
byte perfect_fifth = round(pitch * 1.498307);
byte minor_sixth = round(pitch * 1.587401);
byte major_sixth = round(pitch * 1.681793);
byte minor_seventh = round(pitch * 1.781797);
byte major_seventh = round(pitch * 1.887749);
byte octave = round(pitch * 2.0);

uint16_t index1, index2, index3, index4, index5, index6, window_start, window_end;
int sample1, sample2, sample3, sample4, sample5, sample6;
uint32_t accumulator1, accumulator2, accumulator3, accumulator4, accumulator5, accumulator6;

int delay_sample;
byte delay_buffer[800] = {0};
uint16_t delay_buffer_index;
byte delay_active;

byte recording, record_page_index, record_buffer_index;
uint16_t sample_length = 2052;

byte record_buffer[64] = {0};

const byte sample_table[] PROGMEM =
{ 128,127,130,128,131,129,136,141,182,240,246,246,243,244,238,241,229,237,102,-6,11,-2,7,1,6,2,6,5,8,8,11,13,14,17,18,21,22,26,28,33,36,45,54,70,94,
127,168,206,223,233,238,244,244,245,248,249,253,253,252,251,252,251,251,250,248,246,245,242,240,238,236,234,232,229,226,222,216,208,196,176,147,113,
74,44,30,24,17,13,10,7,4,3,2,2,6,4,3,4,4,4,5,6,7,9,11,12,14,16,18,20,23,26,29,35,44,58,83,115,149,186,212,224,231,235,239,239,240,241,242,243,244,
247,248,249,248,248,247,246,245,243,242,240,238,236,235,233,230,227,224,219,213,202,187,166,140,113,82,53,38,30,25,22,17,14,12,10,9,8,7,6,5,5,5,5,6,
6,7,8,9,11,13,14,16,17,19,21,24,26,31,36,45,59,80,109,140,169,194,212,221,227,231,235,237,239,241,243,244,245,242,240,243,243,243,243,242,241,240,
239,238,237,235,234,232,229,228,224,221,216,207,195,178,153,130,106,80,58,44,35,30,27,22,20,18,17,16,15,14,14,14,14,15,15,14,10,11,11,12,13,15,16,17,
19,20,22,25,28,31,37,44,54,68,88,109,129,152,176,195,209,218,223,228,231,234,236,237,239,238,236,237,237,237,237,238,239,239,239,238,238,237,236,235,
234,232,231,229,226,224,220,214,205,193,174,152,131,111,90,70,54,44,38,30,27,24,22,21,20,19,18,18,18,18,18,19,19,20,19,19,19,20,20,22,23,24,25,27,29,
32,36,41,46,55,67,80,97,113,130,148,167,185,198,208,215,220,224,227,229,231,232,233,234,234,235,235,233,232,232,233,234,234,234,233,232,231,229,228,
226,224,221,218,213,206,196,185,170,154,138,123,108,93,79,67,62,50,40,36,32,30,28,27,25,24,24,23,23,23,24,25,25,25,26,25,25,25,25,26,27,28,29,31,34,
36,40,45,52,62,72,86,102,116,130,145,161,176,188,198,208,213,217,220,222,224,225,227,227,228,228,228,228,228,227,225,223,224,223,223,225,224,223,222,
220,218,214,212,208,201,196,189,177,169,157,146,134,123,111,102,95,75,63,56,49,44,40,37,35,33,32,31,30,30,30,30,30,32,35,34,34,34,33,33,33,33,34,36,
37,41,44,48,55,61,67,76,86,99,110,121,131,141,152,162,172,184,192,199,205,210,213,216,218,220,221,221,222,222,222,221,221,216,214,214,213,213,212,211,
210,209,206,204,205,203,199,195,190,184,175,168,161,151,143,138,126,115,109,99,92,84,77,70,65,60,57,53,50,47,45,44,43,42,42,42,41,43,43,43,44,44,45,
45,45,46,46,46,46,49,51,57,63,71,80,87,94,103,112,121,130,139,150,160,168,176,182,188,193,197,201,204,206,208,210,211,210,210,209,209,208,205,204,204,
204,203,202,201,200,199,195,192,189,185,182,177,171,164,157,149,142,134,127,119,112,105,99,94,89,85,81,77,74,71,69,68,66,64,63,62,61,60,59,58,58,58,
57,57,58,59,60,60,61,63,65,69,73,77,81,86,90,97,103,108,114,119,125,131,136,141,146,150,155,159,164,168,171,174,176,179,181,183,184,186,188,189,190,
191,191,191,189,188,187,186,185,183,182,180,178,175,172,169,166,161,156,152,149,146,140,135,130,125,120,115,112,108,104,101,98,95,91,88,85,82,79,77,
75,73,73,72,71,71,71,71,71,72,71,71,72,74,76,76,78,82,86,89,93,98,100,104,107,112,117,120,124,128,132,136,140,144,148,151,154,157,160,162,164,166,
167,169,170,171,172,173,174,174,174,175,175,175,175,175,174,174,172,170,168,166,164,162,160,156,152,150,145,142,139,135,132,128,125,122,118,116,113,
110,108,105,102,100,97,95,93,91,89,87,86,84,83,82,81,81,81,82,82,83,84,85,87,88,90,93,95,97,100,103,106,110,113,117,121,124,127,130,132,135,137,140,
142,144,146,147,149,151,152,154,155,157,159,160,162,163,164,164,165,165,165,165,164,164,163,163,161,160,159,158,157,155,152,150,148,145,142,139,137,
135,132,130,128,126,124,122,120,118,116,114,112,109,107,105,104,102,101,99,98,97,95,94,94,93,93,94,94,94,95,96,97,98,100,102,104,106,108,110,112,113,
115,116,118,119,121,122,124,126,128,130,132,135,137,139,141,143,146,148,149,151,153,154,155,156,156,156,157,157,157,157,156,156,155,154,153,152,151,
150,149,148,147,146,145,144,142,141,140,138,136,134,132,130,128,126,124,123,121,119,117,115,113,111,109,108,107,106,105,105,105,104,104,104,103,102,
102,102,102,102,102,103,103,104,105,106,107,109,110,111,112,114,116,118,120,122,124,126,127,129,131,132,134,135,137,138,140,141,142,143,143,144,145,
145,146,146,147,148,149,150,150,151,151,152,152,151,151,150,150,149,148,146,144,142,140,138,136,134,132,131,129,128,126,125,123,122,120,119,117,116,
115,114,113,112,112,112,112,112,111,111,111,111,111,111,111,110,110,110,110,110,110,110,111,112,113,114,114,115,115,116,116,117,118,119,121,122,124,
125,127,128,129,130,131,132,133,134,135,136,137,138,139,139,140,141,141,142,142,142,142,142,142,142,142,142,142,142,142,142,141,141,140,140,139,138,
137,137,136,134,133,132,130,129,127,125,124,122,121,119,118,117,116,116,115,114,114,114,113,113,112,112,112,112,112,112,113,113,113,113,113,114,114,
114,114,114,115,115,115,116,116,117,118,119,120,121,122,123,124,126,127,128,130,131,132,134,135,136,137,138,138,139,139,140,140,140,140,141,141,140,
140,140,139,139,139,139,138,138,137,137,136,135,134,133,132,132,131,130,129,128,128,127,126,126,125,124,123,122,121,120,120,119,119,118,118,118,117,
117,116,116,116,116,115,115,115,115,115,115,115,115,115,115,115,116,116,116,117,118,119,120,121,122,122,123,124,125,126,126,127,128,128,129,130,131,
131,131,132,133,134,134,135,135,136,136,136,136,136,136,136,136,136,136,136,136,135,136,136,135,135,135,135,134,133,133,132,131,130,129,128,127,126,
126,125,125,124,124,123,122,122,121,121,121,120,120,119,119,119,119,118,118,118,117,117,117,118,118,118,119,119,120,120,120,120,120,121,121,121,121,
121,121,122,122,123,123,124,124,125,126,127,127,128,129,129,129,130,130,130,130,130,131,131,132,132,133,133,134,134,134,135,135,135,135,135,135,134,
134,133,133,132,132,131,131,131,130,130,130,129,129,128,128,127,127,126,126,125,125,124,124,124,123,123,123,122,122,122,121,121,121,120,120,120,120,
119,119,119,119,119,119,119,119,119,119,119,120,120,121,121,121,122,122,123,123,124,124,125,126,127,127,128,129,129,130,130,130,130,130,130,130,131,
131,132,132,132,133,133,133,134,134,134,134,134,134,134,134,133,133,133,132,132,131,131,131,130,130,129,129,128,128,127,127,126,126,126,125,125,125,
124,124,123,123,122,122,121,121,121,121,121,121,121,121,121,121,121,121,121,121,122,122,122,122,123,123,123,123,123,123,124,124,124,124,124,124,125,
125,125,126,126,126,126,126,127,127,127,128,128,128,129,129,130,130,130,130,131,131,131,131,132,132,132,132,132,132,132,132,132,132,132,132,131,131,
131,130,130,129,129,128,128,128,128,127,127,127,126,126,125,125,125,124,124,123,123,123,123,123,123,123,122,122,122,122,122,122,122,122,122,122,122,
122,122,122,123,123,123,123,124,124,124,124,125,125,125,125,125,126,126,126,126,127,127,127,127,127,128,128,128,129,129,129,130,130,130,130,130,130,
130,130,130,130,130,130,130,130,130,129,129,129,129,129,129,129,128,128,128,128,127,127,127,126,126,126,125,125,125,125,124,124,124,124,124,124,124,
124,124,124,124,124,124,124,124,124,124,124,124,124,124,125,125,125,125,124,124,124,124,124,124,124,125,125,125,125,125,125,125,126,126,126,126,126,
127,127,127,127,127,128,128,128,128,128,128,128,128,129,129,129,129,129,129,129,129,129,129,129,129,129,128,128,128,128,128,127,127,127,127,127,127,
127,127,126,126,126,126,126,126,126,126,126,126,125,125,125,125,125,125,125,125,125,124,124,124,124,124,124,124,124,124,124,124,124,124,124,124,124,
124,124,125,125,125,125,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,128,128,128,128,128,128,128,127,127,127,127,127,127,128,
128,128,128,128,128,128,128,128,128,128,128,128,127,127,127,127,127,127,127,127,127,126,126,126,126,126,125,125,125,125,125,125,125,125,124,124,124,
124,124,124,124,124,124,124,124,124,124,124,124,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,126,126,126,127,127,127,127,127,127,127,
127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,129,129,129,129,128,128,128,128,127,127,127,127,126,126,126,126,126,126,
126,126,126,126,126,125,125,125,125,125,125,125,125,125,125,125,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,126,126,126,126,126,126,
126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127};



void setup(void) {
  cli();

  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  
  // set compare match register for 8khz increments
  OCR2A = 242;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();
}

ISR(TIMER2_COMPA_vect) {
  if (recording) {
    record_buffer[record_buffer_index] = (analogRead(0) & 255);

    record_buffer_index++;

    if (record_buffer_index == 64) {
      write_eeprom_page(record_page_index);
      record_page_index++;
      record_buffer_index = 0;
    }
  } else {
    if (!continuous && s1_trigger) {
      index1 = window_start;
      accumulator1 = 0;
      s1_latch = 1;
      s1_trigger = 0;
    }

    if (!continuous && s2_trigger) {
      index2 = window_start;
      accumulator2 = window_start;
      s2_latch = 1;
      s2_trigger = 0;
    }

    if (!continuous && s3_trigger) {
      index3 = window_start;
      accumulator3 = window_start;
      s3_latch = 1;
      s3_trigger = 0;
    }

    if (!continuous && s4_trigger) {
      index4 = window_start;
      accumulator4 = window_start;
      s4_latch = 1;
      s4_trigger = 0;
    }

    if (!continuous && s5_trigger) {
      index5 = window_start;
      accumulator5 = window_start;
      s5_latch = 1;
      s5_trigger = 0;
    }

    if (!continuous && s6_trigger) {
      index6 = window_start;
      accumulator6 = window_start;
      s6_latch = 1;
      s6_trigger = 0;
    }

    // sample1 = (pgm_read_byte(&sample_table[index1]) - 127) * (s1_latch || (continuous && !s1));
    sample1 = (read_eeprom_byte(index1) - 127) * (s1_latch || (continuous && !s1));
    // sample2 = (pgm_read_byte(&sample_table[index2]) - 127) * (s2_latch || (continuous && !s2));
    sample2 = (read_eeprom_byte(index2) - 127) * (s2_latch || (continuous && !s2));
    // sample3 = (pgm_read_byte(&sample_table[index3]) - 127) * (s3_latch || (continuous && !s3));
    // sample4 = (pgm_read_byte(&sample_table[index4]) - 127) * (s4_latch || (continuous && !s4));
    // sample5 = (pgm_read_byte(&sample_table[index5]) - 127) * (s5_latch || (continuous && !s5));
    // sample6 = (pgm_read_byte(&sample_table[index6]) - 127) * (s6_latch || (continuous && !s6));
    // if (s1_latch || (continuous && !s1)) {
      // read from eeprom
      // sample1 = read_eeprom_byte(index1) - 127;
      // Serial.println(sample1);
    // }

    delay_sample = delay_buffer[delay_buffer_index] * delay_active;

    sample_out_temp = ((sample1 + sample2 + sample3 + sample4 + sample5 + sample6 + delay_sample) >> 1) + 127;

    if (sample_out_temp > 255) {
      sample_out_temp -= (sample_out_temp - 255) << 1;
    }

    if (sample_out_temp < 0) {
      sample_out_temp += sample_out_temp * -2;
    }

    sample_out = sample_out_temp;

    uint16_t dac_out = (0 << 15) | (1 << 14) | (1 << 13) | (1 << 12) | (sample_out << 4);
    digitalWrite(10, LOW);
    SPI.transfer(dac_out >> 8);
    SPI.transfer(dac_out & 255);
    digitalWrite(10, HIGH);

    delay_buffer[delay_buffer_index] = (sample_out >> 1);

    if (continuous || s1_latch) {
      accumulator1 += pitch;
      index1 = window_start + (accumulator1 >> (6));

      if (index1 > window_end) {
        index1 = window_start;
        accumulator1 = 0;
        s1_latch = 0;
      }
    }

    if (continuous || s2_latch) {
      accumulator2 += minor_second;
      index2 = window_start + (accumulator2 >> (6));

      if (index2 > window_end) {
        index2 = window_start;
        accumulator2 = window_start;
        s2_latch = 0;
      }
    }

    if (continuous || s3_latch) {
      accumulator3 += major_second;
      index3 = (accumulator3 >> (6));

      if (index3 > window_end) {
        index3 = window_start;
        accumulator3 = window_start;
        s3_latch = 0;
      }
    }

    if (continuous || s4_latch) {
      accumulator4 += minor_third;
      index4 = (accumulator4 >> (6));

      if (index4 > window_end) {
        index4 = window_start;
        accumulator4 = window_start;
        s4_latch = 0;
      }
    }

    if (continuous || s5_latch) {
      accumulator5 += major_third;
      index5 = (accumulator5 >> (6));

      if (index5 > window_end) {
        index5 = window_start;
        accumulator5 = window_start;
        s5_latch = 0;
      }
    }

    if (continuous || s6_latch) {
      accumulator6 += perfect_fourth;
      index6 = (accumulator6 >> (6));

      if (index6 > window_end) {
        index6 = window_start;
        accumulator6 = window_start;
        s6_latch = 0;
      }
    }

    delay_buffer_index++;
    if (delay_buffer_index == 800) delay_buffer_index = 0;
  }
}

void loop(void) {
  // analogRead should be able to do 0-1023 for values
  // but my potentiometers only get up to ~855
  window_start = map(analogRead(A6), 0, 855, 0, sample_length);
  window_end   = map(analogRead(A7), 0, 855, 0, sample_length);
  
  byte d1 = digitalRead(2);
  byte d2 = digitalRead(3);
  byte d3 = digitalRead(4);
  byte d4 = digitalRead(5);
  byte d5 = digitalRead(6);
  byte d6 = digitalRead(7);
  byte d7 = digitalRead(8);

  if (recording) {
    // account for page 0 with this "+ 1"
    if ((record_page_index + 1) * PAGE_SIZE > RECORD_LIMIT) {
      d1 = 1;
      Serial.println("really okay");
    }
  }

  if (d1 == ls1) ld1 = millis();
  if (d2 == ls2) ld2 = millis();
  if (d3 == ls3) ld3 = millis();
  if (d4 == ls4) ld4 = millis();
  if (d5 == ls5) ld5 = millis();
  if (d6 == ls6) ld6 = millis();
  if (d7 == ls7) ld7 = millis();

  if (d1 != ls1 && (millis() - ld1) > debounceDelay) {
    s1 = d1;

    if (!s7) {
      if (!s1) {
        record_buffer_index = 0;
        record_page_index = 0;
        recording = 1;
      } else {
        recording = 0;
        if (record_page_index > 0) {
          sample_length = record_page_index * PAGE_SIZE;
        }
        Serial.println(sample_length);
      }
    } else {
      s1_trigger = !s1 && ls1 && !continuous;
    }
  }

  if (d2 != ls2 && (millis() - ld2) > debounceDelay) {
    s2 = d2;

    if (!s7 && !s2) {
      delay_active = !delay_active;
      delay_buffer_index = 0;
    } else {
      s2_trigger = !s2 && ls2 && !continuous;
    }
  }

  if (d3 != ls3 && (millis() - ld3) > debounceDelay) {
    s3 = d3;

    if (!s7 && !s3) {
      continuous = !continuous;
      index1 = window_start;
      index2 = window_start;
      index3 = window_start;
      index4 = window_start;
      index5 = window_start;

      accumulator1 = 0;
      accumulator2 = window_start;
      accumulator3 = window_start;
      accumulator4 = window_start;
      accumulator5 = window_start;
    } else {
      s3_trigger = !s3 && ls3 && !continuous;
    }
  }

  if (d4 != ls4 && (millis() - ld4) > debounceDelay) {
    s4 = d4;
    s4_trigger = !s4 && ls4 && !continuous;
  }

  if (d5 != ls5 && (millis() - ld5) > debounceDelay) {
    s5 = d5;
    s5_trigger = !s5 && ls5 && !continuous;
  }

  if (d6 != ls6 && (millis() - ld6) > debounceDelay) {
    s6 = d6;
    s6_trigger = !s6 && ls6 && !continuous;
  }

  if (d7 != ls7 && (millis() - ld7) > debounceDelay) {
    s7 = d7;
  }
  
  ls1 = s1;
  ls2 = s2;
  ls3 = s3;
  ls4 = s4;
  ls5 = s5;
  ls6 = s6;
  ls7 = s7;
}

bool write_eeprom_page(uint16_t page_index) {
  uint16_t address = page_index * PAGE_SIZE;

  enable_eeprom_write();

  // CS low
  digitalWrite(9, LOW);

  // instruction
  SPI.transfer(WRITE);

  // MSB of address
  SPI.transfer(address >> 8);

  // LSB of address
  SPI.transfer(address & 255);

  int i = 0;
  for (i; i < 64; i++) {
    SPI.transfer(record_buffer[i]);
  }

  // bring CS high to end SPI interaction
  digitalWrite(9, HIGH);

  delay(5);

  return true;
}

byte read_eeprom_byte(uint16_t address) {
  // CS low
  digitalWrite(9, LOW);

  // read instruction
  SPI.transfer(READ);

  // MSB of address
  SPI.transfer(address >> 8);

  // LSB of address
  SPI.transfer(address & 255);

  byte val = SPI.transfer(1);

  // CS back high
  digitalWrite(9, HIGH);

  return val;
}

bool enable_eeprom_write() {
  digitalWrite(9, LOW);
  SPI.transfer(WRITE_ENABLE);
  digitalWrite(9, HIGH);
  delay(5);
  return true;
}
