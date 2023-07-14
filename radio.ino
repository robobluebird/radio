#include <SPI.h>

const byte sine_table[] PROGMEM = { 128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201,
                                    203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249,
                                    250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249,
                                    248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203, 201,
                                    198, 196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 128, 124,
                                    121, 118, 115, 112, 109, 106, 103, 100, 97, 93, 90, 88, 85, 82, 79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40, 37,
                                    35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2,
                                    2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62,
                                    65, 67, 70, 73, 76, 79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124 };

#define EEPROM_WRITE_ENABLE 6
#define EEPROM_WRITE 2
#define EEPROM_READ 3
#define EEPROM_PAGE_SIZE 64

#define P1 0b10000000
#define P2 0b01000000
#define P3 0b00100000
#define P4 0b00010000
#define P5 0b00001000
#define P6 0b00000100
#define P7 0b00000010
#define P8 0b00000001

#define NP1 0b01111111
#define NP2 0b10111111
#define NP3 0b11011111
#define NP4 0b11101111
#define NP5 0b11110111
#define NP6 0b11111011
#define NP7 0b11111101
#define NP8 0b11111110

byte s1, s1_trigger, s1_latch, bd1;
byte ls1 = HIGH;
unsigned long ld1 = 0;

byte s2, s2_trigger, s2_latch, bd2;
byte ls2 = HIGH;
unsigned long ld2 = 0;

byte s3, s3_trigger, s3_latch, bd3;
byte ls3 = HIGH;
unsigned long ld3 = 0;

byte s4, s4_trigger, s4_latch, bd4;
byte ls4 = HIGH;
unsigned long ld4 = 0;

byte s5, s5_trigger, s5_latch, bd5;
byte ls5 = HIGH;
unsigned long ld5 = 0;

byte s6, s6_trigger, s6_latch, bd6;
byte ls6 = HIGH;
unsigned long ld6 = 0;

byte s7, s7_trigger, s7_latch, bd7;
byte ls7 = HIGH;
unsigned long ld7 = 0;

byte s8, s8_trigger, s8_latch, bd8;
byte ls8 = HIGH;
unsigned long ld8 = 0;

byte s9, s9_trigger, s9_latch, bd9;
byte ls9 = HIGH;
unsigned long ld9 = 0;

byte s10, s10_trigger, s10_latch, bd10;
byte ls10 = HIGH;
unsigned long ld10 = 0;

byte s11, s11_trigger, s11_latch, bd11;
byte ls11 = HIGH;
unsigned long ld11 = 0;

byte s12, s12_trigger, s12_latch, bd12;
byte ls12 = HIGH;
unsigned long ld12 = 0;

byte s13, s13_trigger, s13_latch, bd13;
byte ls13 = HIGH;
unsigned long ld13 = 0;

byte sf;
byte lsf = HIGH;
unsigned long ldf = 0;

byte continuous, reverse, boomerang, am = 0;

const unsigned long debounceDelay = 50;

int sample_out_temp;
byte sample_out;

byte pitch = 60;
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

const unsigned long dds_tune = 4294967296 / 8000;  // dds thing, don't worry about it

uint16_t index1, index2, index3, index4, index5, index6, index7, index8, index9, index10, index11, index12, index13, sine_index, window_start, window_end;
int sample1, sample2, sample3, sample4, sample5, sample6, sample7, sample8, sample9, sample10, sample11, sample12, sample13;
uint32_t accumulator1, accumulator2, accumulator3, accumulator4, accumulator5, accumulator6, accumulator7, accumulator8, accumulator9, accumulator10, accumulator11, accumulator12, accumulator13, sine_accumulator;

int delay_sample;
byte delay_buffer[1000] = { 0 };
uint16_t delay_buffer_index;
byte delay_active;

byte record_buffer_index, recording;
uint16_t sample_length, record_page_index;

byte record_buffer[64] = { 0 };

byte initial_adcsra, initial_admux;
byte p;       // keeps count of how many samples are playing at any one time
byte mp = 3;  // max number of samples that can be playing at one time (based on speed of byte read from storage...too many and it'll choke!)
byte pr;      // "playing register" -> use bits to track which pitches are active to avoid doubling the count...just do 8 for now...

void setup(void) {
  cli();

  // Serial.begin(9600);

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  // CS pin for eeprom
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  SPI.begin();

  TCCR2A = 0;  // set entire TCCR2A register to 0
  TCCR2B = 0;  // same for TCCR2B
  TCNT2 = 0;   // initialize counter value to 0

  // set compare match register for 8khz increments
  // OCR2A = 207; // = (16*10^6) / (8000*8) - 1 (must be < 256)
  OCR2A = 249;
  TCCR2A |= (1 << WGM21);   // turn on CTC mode
  TCCR2B |= (1 << CS21);    // Set CS21 bit for 8 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt

  // capture initial values of ADC registers to restore when recording halts
  initial_adcsra = ADCSRA;
  initial_admux = ADMUX;

  sei();
}

ISR(TIMER2_COMPA_vect) {
  if (!continuous && s1_trigger) {
    if (pr & P1) {
      if (reverse) {
        index1 = window_end;
      } else {
        index1 = window_start;
      }
    } else if (p < mp) {
      if (reverse) {
        index1 = window_end;
      } else {
        index1 = window_start;
      }

      pr |= P1;
      p++;
    }

    accumulator1 = 0;
    s1_latch = 1;
    s1_trigger = 0;
  }

  if (!continuous && s2_trigger) {
    if (pr & P2) {
      if (reverse) {
        index2 = window_end;
      } else {
        index2 = window_start;
      }

    } else if (p < mp) {
      if (reverse) {
        index2 = window_end;
      } else {
        index2 = window_start;
      }

      pr |= P2;
      p++;
    }

    accumulator2 = 0;
    s2_latch = 1;
    s2_trigger = 0;
  }

  if (!continuous && s3_trigger) {
    if (pr & P3) {
      if (reverse) {
        index3 = window_end;
      } else {
        index3 = window_start;
      }
    } else if (p < mp) {
      if (reverse) {
        index3 = window_end;
      } else {
        index3 = window_start;
      }

      pr |= P3;
      p++;
    }

    accumulator3 = 0;
    s3_latch = 1;
    s3_trigger = 0;
  }

  if (!continuous && s4_trigger) {
    if (pr & P4) {
      if (reverse) {
        index4 = window_end;
      } else {
        index4 = window_start;
      }

    } else if (p < mp) {
      if (reverse) {
        index4 = window_end;
      } else {
        index4 = window_start;
      }

      pr |= P4;
      p++;
    }

    accumulator4 = 0;
    s4_latch = 1;
    s4_trigger = 0;
  }

  if (!continuous && s5_trigger) {
    if (pr & P5) {
      if (reverse) {
        index5 = window_end;
      } else {
        index5 = window_start;
      }
    } else if (p < mp) {
      if (reverse) {
        index5 = window_end;
      } else {
        index5 = window_start;
      }

      pr |= P5;
      p++;
    }

    accumulator5 = 0;
    s5_latch = 1;
    s5_trigger = 0;
  }

  if (!continuous && s6_trigger) {
    if (pr & P6) {
      if (reverse) {
        index6 = window_end;
      } else {
        index6 = window_start;
      }
    } else if (p < mp) {
      if (reverse) {
        index6 = window_end;
      } else {
        index6 = window_start;
      }

      pr |= P6;
      p++;
    }

    accumulator6 = 0;
    s6_latch = 1;
    s6_trigger = 0;
  }

  if (!continuous && s7_trigger) {
    if (pr & P7) {
      if (reverse) {
        index7 = window_end;
      } else {
        index7 = window_start;
      }
    } else if (p < mp) {
      if (reverse) {
        index7 = window_end;
      } else {
        index7 = window_start;
      }

      pr |= P7;
      p++;
    }

    accumulator7 = 0;
    s7_latch = 1;
    s7_trigger = 0;
  }

  if (!continuous && s8_trigger) {
    if (pr & P8) {
      if (reverse) {
        index8 = window_end;
      } else {
        index8 = window_start;
      }
    } else if (p < mp) {
      if (reverse) {
        index8 = window_end;
      } else {
        index8 = window_start;
      }

      pr |= P8;
      p++;
    }

    accumulator8 = 0;
    s8_latch = 1;
    s8_trigger = 0;
  }

  if (s1_latch || (continuous && (pr & P1))) {
    sample1 = read_eeprom_byte(index1) - 127;
  } else {
    sample1 = 0;
  }

  if (s2_latch || (continuous && (pr & P2))) {
    sample2 = read_eeprom_byte(index2) - 127;
  } else {
    sample2 = 0;
  }

  if (s3_latch || (continuous && (pr & P3))) {
    sample3 = read_eeprom_byte(index3) - 127;
  } else {
    sample3 = 0;
  }

  if (s4_latch || (continuous && (pr & P4))) {
    sample4 = read_eeprom_byte(index4) - 127;
  } else {
    sample4 = 0;
  }

  if (s5_latch || (continuous && (pr & P5))) {
    sample5 = read_eeprom_byte(index5) - 127;
  } else {
    sample5 = 0;
  }

  if (s6_latch || (continuous && (pr & P6))) {
    sample6 = read_eeprom_byte(index6) - 127;
  } else {
    sample6 = 0;
  }

  if (s7_latch || (continuous && (pr & P7))) {
    sample7 = read_eeprom_byte(index7) - 127;
  } else {
    sample7 = 0;
  }

  if (s8_latch || (continuous && (pr & P8))) {
    sample8 = read_eeprom_byte(index8) - 127;
  } else {
    sample8 = 0;
  }

  // later: try using signed byte for samples (-127...127)

  delay_sample = delay_buffer[delay_buffer_index] * delay_active;

  if (am) {
    // fill the rest of this in somehow, some way
    sample_out_temp = ((int)((pgm_read_byte(&sine_table[sine_index]) - 127) * (3 * (sample1 / 127.0))) >> 1) + 127;
  } else {
    sample_out_temp = ((sample1 + sample2 + sample3 + sample4 + sample5 + sample6 + sample7 + sample8 + delay_sample) >> 1) + 127;
  }

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

  if (am) {
    sine_accumulator += 220 << 2;
    sine_index = (dds_tune * sine_accumulator) >> (32 - 8);
  }

  if (continuous || s1_latch) {
    accumulator1 += pitch;

    if (reverse) {
      index1 = window_end - (accumulator1 >> (6));

      if (index1 < window_start) {
        index1 = window_end;
        accumulator1 = 0;
        s1_latch = 0;

        if (!continuous) {
          pr &= NP1;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd1) {
        index1 = window_end - (accumulator1 >> (6));

        if (index1 < window_start) {
          index1 = window_start;
          accumulator1 = 0;
          s1_latch = 0;
          bd1 = 0;

          if (!continuous) {
            pr &= NP1;
            p--;
          }
        }
      } else {
        index1 = window_start + (accumulator1 >> (6));

        if (index1 > window_end) {
          bd1 = 1;
          index1 = window_end;
          accumulator1 = 0;
        }
      }
    } else {
      index1 = window_start + (accumulator1 >> (6));

      if (index1 > window_end) {
        index1 = window_start;
        accumulator1 = 0;
        s1_latch = 0;

        if (!continuous) {
          pr &= NP1;
          p--;
        }
      }
    }
  }

  if (continuous || s2_latch) {
    accumulator2 += minor_second;

    if (reverse) {
      index2 = window_end - (accumulator2 >> (6));

      if (index2 < window_start) {
        index2 = window_end;
        accumulator2 = 0;
        s2_latch = 0;

        if (!continuous) {
          pr &= NP2;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd2) {
        index2 = window_end - (accumulator2 >> (6));

        if (index2 < window_start) {
          index2 = window_start;
          accumulator2 = 0;
          s2_latch = 0;
          bd2 = 0;

          if (!continuous) {
            pr &= NP2;
            p--;
          }
        }
      } else {
        index2 = window_start + (accumulator2 >> (6));

        if (index2 > window_end) {
          bd2 = 1;
          index2 = window_end;
          accumulator2 = 0;
        }
      }
    } else {
      index2 = window_start + (accumulator2 >> (6));

      if (index2 > window_end) {
        index2 = window_start;
        accumulator2 = 0;
        s2_latch = 0;

        if (!continuous) {
          pr &= NP2;
          p--;
        }
      }
    }
  }

  if (continuous || s3_latch) {
    accumulator3 += major_second;

    if (reverse) {
      index3 = window_end - (accumulator3 >> (6));

      if (index3 < window_start) {
        index3 = window_end;
        accumulator3 = 0;
        s3_latch = 0;

        if (!continuous) {
          pr &= NP3;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd3) {
        index3 = window_end - (accumulator3 >> (6));

        if (index3 < window_start) {
          index3 = window_start;
          accumulator3 = 0;
          s3_latch = 0;
          bd3 = 0;

          if (!continuous) {
            pr &= NP3;
            p--;
          }
        }
      } else {
        index3 = window_start + (accumulator3 >> (6));

        if (index3 > window_end) {
          bd3 = 1;
          index3 = window_end;
          accumulator3 = 0;
        }
      }
    } else {
      index3 = window_start + (accumulator3 >> (6));

      if (index3 > window_end) {
        index3 = window_start;
        accumulator3 = 0;
        s3_latch = 0;

        if (!continuous) {
          pr &= NP3;
          p--;
        }
      }
    }
  }

  if (continuous || s4_latch) {
    accumulator4 += minor_third;

    if (reverse) {
      index4 = window_end - (accumulator4 >> (6));

      if (index4 < window_start) {
        index4 = window_end;
        accumulator4 = 0;
        s4_latch = 0;

        if (!continuous) {
          pr &= NP4;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd4) {
        index4 = window_end - (accumulator4 >> (6));

        if (index4 < window_start) {
          index4 = window_start;
          accumulator4 = 0;
          s4_latch = 0;
          bd4 = 0;

          if (!continuous) {
            pr &= NP4;
            p--;
          }
        }
      } else {
        index4 = window_start + (accumulator4 >> (6));

        if (index4 > window_end) {
          bd4 = 1;
          index4 = window_end;
          accumulator4 = 0;
        }
      }
    } else {
      index4 = window_start + (accumulator4 >> (6));

      if (index4 > window_end) {
        index4 = window_start;
        accumulator4 = 0;
        s4_latch = 0;

        if (!continuous) {
          pr &= NP4;
          p--;
        }
      }
    }
  }

  if (continuous || s5_latch) {
    accumulator5 += major_third;

    if (reverse) {
      index5 = window_end - (accumulator5 >> (6));

      if (index5 < window_start) {
        index5 = window_end;
        accumulator5 = 0;
        s5_latch = 0;

        if (!continuous) {
          pr &= NP5;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd5) {
        index5 = window_end - (accumulator5 >> (6));

        if (index5 < window_start) {
          index5 = window_start;
          accumulator5 = 0;
          s5_latch = 0;
          bd5 = 0;

          if (!continuous) {
            pr &= NP5;
            p--;
          }
        }
      } else {
        index5 = window_start + (accumulator5 >> (6));

        if (index5 > window_end) {
          bd5 = 1;
          index5 = window_end;
          accumulator5 = 0;
        }
      }
    } else {
      index5 = window_start + (accumulator5 >> (6));

      if (index5 > window_end) {
        index5 = window_start;
        accumulator5 = 0;
        s5_latch = 0;

        if (!continuous) {
          pr &= NP5;
          p--;
        }
      }
    }
  }

  if (continuous || s6_latch) {
    accumulator6 += perfect_fourth;

    if (reverse) {
      index6 = window_end - (accumulator6 >> (6));

      if (index6 < window_start) {
        index6 = window_end;
        accumulator6 = 0;
        s6_latch = 0;

        if (!continuous) {
          pr &= NP6;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd6) {
        index6 = window_end - (accumulator6 >> (6));

        if (index6 < window_start) {
          index6 = window_start;
          accumulator6 = 0;
          s6_latch = 0;
          bd6 = 0;

          if (!continuous) {
            pr &= NP6;
            p--;
          }
        }
      } else {
        index6 = window_start + (accumulator6 >> (6));

        if (index6 > window_end) {
          bd6 = 1;
          index6 = window_end;
          accumulator6 = 0;
        }
      }
    } else {
      index6 = window_start + (accumulator6 >> (6));

      if (index6 > window_end) {
        index6 = window_start;
        accumulator6 = 0;
        s6_latch = 0;

        if (!continuous) {
          pr &= NP6;
          p--;
        }
      }
    }
  }

  if (continuous || s7_latch) {
    accumulator7 += tritone;

    if (reverse) {
      index7 = window_end - (accumulator7 >> (6));

      if (index7 < window_start) {
        index7 = window_end;
        accumulator7 = 0;
        s7_latch = 0;

        if (!continuous) {
          pr &= NP7;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd7) {
        index7 = window_end - (accumulator7 >> (6));

        if (index7 < window_start) {
          index7 = window_start;
          accumulator7 = 0;
          s7_latch = 0;
          bd7 = 0;

          if (!continuous) {
            pr &= NP7;
            p--;
          }
        }
      } else {
        index7 = window_start + (accumulator7 >> (6));

        if (index7 > window_end) {
          bd7 = 1;
          index7 = window_end;
          accumulator7 = 0;
        }
      }
    } else {
      index7 = window_start + (accumulator7 >> (6));

      if (index7 > window_end) {
        index7 = window_start;
        accumulator7 = 0;
        s7_latch = 0;

        if (!continuous) {
          pr &= NP7;
          p--;
        }
      }
    }
  }

  if (continuous || s8_latch) {
    accumulator8 += perfect_fifth;

    if (reverse) {
      index8 = window_end - (accumulator8 >> (6));

      if (index8 < window_start) {
        index8 = window_end;
        accumulator8 = 0;
        s8_latch = 0;

        if (!continuous) {
          pr &= NP8;
          p--;
        }
      }
    } else if (boomerang) {
      if (bd8) {
        index8 = window_end - (accumulator8 >> (6));

        if (index8 < window_start) {
          index8 = window_start;
          accumulator8 = 0;
          s8_latch = 0;
          bd8 = 0;

          if (!continuous) {
            pr &= NP8;
            p--;
          }
        }
      } else {
        index8 = window_start + (accumulator8 >> (6));

        if (index8 > window_end) {
          bd8 = 1;
          index8 = window_end;
          accumulator8 = 0;
        }
      }
    } else {
      index8 = window_start + (accumulator8 >> (6));

      if (index8 > window_end) {
        index8 = window_start;
        accumulator8 = 0;
        s8_latch = 0;

        if (!continuous) {
          pr &= NP8;
          p--;
        }
      }
    }
  }

  delay_buffer_index++;

  if (delay_buffer_index == 1000) delay_buffer_index = 0;
}

void enable_record() {
  cli();  // disable interrupt for playback timer

  TIMSK2 |= (0 << OCIE2A);  // disable playback

  ADCSRA = 0;  // clear adc "a" register

  // set ADMUX to inscrutible value... 01100000... ref voltage to vref, left shift capture, read from A0
  // setting it this way because I don't know where to find the arduino constants for ADMUX[3:0] :-)
  ADMUX = 96;

  // set ADC clock with 128 prescaler -> 16 mHz/128 = 125 kHz
  // 125 kHz / 13 clock cycles per sample (minus first sample) = ~9615 samples per second
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRA |= (1 << ADATE);  // enable auto trigger
  ADCSRA |= (1 << ADIE);   // enable interrupts
  ADCSRA |= (1 << ADEN);   // enable ADC
  ADCSRA |= (1 << ADSC);   // start ADC measurements

  sei();
}

void disable_record() {
  cli();

  // reset ADC register values to...defaults??
  ADCSRA = initial_adcsra;
  ADMUX = initial_admux;

  if (record_page_index > 0) {
    sample_length = record_page_index * EEPROM_PAGE_SIZE;  // clip to last full page

    int i = 0;
    for (i; i < 64; i++) {
      record_buffer[i] = 0;
    }

    record_buffer[0] = (sample_length >> 8);
    record_buffer[1] = (sample_length & 255);

    write_eeprom_page(3999);
  }

  TIMSK2 |= (1 << OCIE2A);  // enable playback

  pr = 0;

  sei();
}

ISR(ADC_vect) {
  record_buffer[record_buffer_index] = ADCH;

  record_buffer_index++;

  if (record_buffer_index == 64) {
    write_eeprom_page(record_page_index);
    record_page_index++;
    record_buffer_index = 0;
  }
}

void loop(void) {
  byte d1 = digitalRead(2);
  byte d2 = digitalRead(3);
  byte d3 = digitalRead(4);
  byte d4 = digitalRead(5);
  byte d5 = digitalRead(6);
  byte d6 = digitalRead(7);
  byte d7 = digitalRead(1);
  byte d8 = digitalRead(0);

  byte df = digitalRead(8);

  if (d1 == ls1) {
    if (continuous && !s1 && (pr & NP1) && p < mp) {
      pr |= P1;
      p++;
    }

    ld1 = millis();
  } else if (millis() - ld1 > debounceDelay) {
    s1 = d1;

    if (!sf) {
      if (!s1) {
        recording = 1;
        record_buffer_index = 0;
        record_page_index = 0;
        enable_record();
      } else {
        disable_record();
        recording = 0;
      }
    } else {
      if (continuous) {
        if (!s1) {
          if (p < mp) {
            pr |= P1;
            p++;
          }
        } else if (pr & P1) {
          pr &= NP1;
          p--;
        }
      } else {
        s1_trigger = !s1 && ls1;
      }
    }
  }

  if (d2 == ls2) {
    if (continuous && !s2 && (pr & NP2) && p < mp) {
      pr |= P2;
      p++;
    }

    ld2 = millis();
  } else if (millis() - ld2 > debounceDelay) {
    s2 = d2;

    if (!sf && !s2) {
      delay_active = !delay_active;
      delay_buffer_index = 0;
    } else {
      if (continuous) {
        if (!s2) {
          if (p < mp) {
            pr |= P2;
            p++;
          }
        } else if (pr & P2) {
          pr &= NP2;
          p--;
        }
      } else {
        s2_trigger = !s2;
      }
    }
  }


  if (d3 == ls3) {
    if (continuous && !s3 && (pr & NP3) && p < mp) {
      pr |= P3;
      p++;
    }

    ld3 = millis();
  } else if (millis() - ld3 > debounceDelay) {
    s3 = d3;

    if (!sf && !s3) {
      continuous = !continuous;

      if (continuous) {
        mp = 2;
      } else {
        mp = 3;
      }

      reset_indexes_and_accumulators(0);
    } else {
      if (continuous) {
        if (!s3) {
          if (p < mp) {
            pr |= P3;
            p++;
          }
        } else if (pr & P3) {
          pr &= NP3;
          p--;
        }
      } else {
        s3_trigger = !s3 && ls3;
      }
    }
  }

  if (d4 == ls4) {
    if (continuous && !s4 && (pr & NP4) && p < mp) {
      pr |= P4;
      p++;
    }

    ld4 = millis();
  } else if (millis() - ld4 > debounceDelay) {
    s4 = d4;

    if (!sf && !s4) {
      reverse = !reverse;

      if (boomerang) boomerang = 0;

      reset_indexes_and_accumulators(1);
    } else {
      if (continuous) {
        if (!s4) {
          if (p < mp) {
            pr |= P4;
            p++;
          }
        } else if (pr & P4) {
          pr &= NP4;
          p--;
        }
      } else {
        s4_trigger = !s4 && ls4;
      }
    }
  }

  if (d5 == ls5) {
    if (continuous && !s5 && (pr & NP5) && p < mp) {
      pr |= P5;
      p++;
    }

    ld5 = millis();
  } else if (millis() - ld5 > debounceDelay) {
    s5 = d5;

    if (!sf && !s5) {
      boomerang = !boomerang;

      if (reverse) reverse = 0;

      reset_indexes_and_accumulators(0);
    } else {
      if (continuous) {
        if (!s5) {
          if (p < mp) {
            pr |= P5;
            p++;
          }
        } else if (pr & P5) {
          pr &= NP5;
          p--;
        }
      } else {
        s5_trigger = !s5 && ls5;
      }
    }
  }

  if (d6 == ls6) {
    if (continuous && !s6 && (pr & NP6) && p < mp) {
      pr |= P6;
      p++;
    }

    ld6 = millis();
  } else if (millis() - ld6 > debounceDelay) {
    s6 = d6;

    if (!sf && !s6) {
      am = !am;

      reset_indexes_and_accumulators(0);
    } else {
      if (continuous) {
        if (!s6) {
          if (p < mp) {
            pr |= P6;
            p++;
          }
        } else if (pr & P6) {
          pr &= NP6;
          p--;
        }
      } else {
        s6_trigger = !s6 && ls6;
      }
    }
  }

  if (d7 == ls7) {
    if (continuous && !s7 && (pr & NP7) && p < mp) {
      pr |= P7;
      p++;
    }

    ld7 = millis();
  } else if (millis() - ld7 > debounceDelay) {
    s7 = d7;

    if (continuous) {
      if (!s7) {
        if (p < mp) {
          pr |= P7;
          p++;
        }
      } else if (pr & P7) {
        pr &= NP7;
        p--;
      }
    } else {
      s7_trigger = !s7 && ls7;
    }
  }

  if (d8 == ls8) {
    if (continuous && !s8 && (pr & NP8) && p < mp) {
      pr |= P8;
      p++;
    }

    ld8 = millis();
  } else if (millis() - ld8 > debounceDelay) {
    s8 = d8;

    if (continuous) {
      if (!s8) {
        if (p < mp) {
          pr |= P8;
          p++;
        }
      } else if (pr & P8) {
        pr &= NP8;
        p--;
      }
    } else {
      s8_trigger = !s8 && ls8;
    }
  }

  if (df == lsf) {
    ldf = millis();
  } else if (millis() - ldf > debounceDelay) {
    sf = df;
  }

  ls1 = s1;
  ls2 = s2;
  ls3 = s3;
  ls4 = s4;
  ls5 = s5;
  ls6 = s6;
  ls7 = s7;
  ls8 = s8;

  lsf = sf;

  // analogRead should be able to do 0-1023 for values
  // but my potentiometers only get up to ~855
  //
  // ALSO only take analogReads when not recording, they won't
  // work when recording because of the adc register configuration
  if (!recording) {
    window_start = map(analogRead(A6), 0, 855, 0, sample_length);
    window_end = map(analogRead(A7), 0, 855, 0, sample_length);
  }
}

void reset_indexes_and_accumulators(byte reverse) {
  if (reverse) {
    index1 = window_end;
    index2 = window_end;
    index3 = window_end;
    index4 = window_end;
    index5 = window_end;
    index6 = window_end;
  } else {
    index1 = window_start;
    index2 = window_start;
    index3 = window_start;
    index4 = window_start;
    index5 = window_start;
    index6 = window_start;
  }

  accumulator1 = 0;
  accumulator2 = 0;
  accumulator3 = 0;
  accumulator4 = 0;
  accumulator5 = 0;
  accumulator6 = 0;
}

void write_ram_byte(uint16_t address) {
}

void write_ram_page(uint16_t page_index) {
}

void read_ram_byte(uint16_t address) {
}

void write_eeprom_page(uint16_t page_index) {
  uint16_t address = page_index * EEPROM_PAGE_SIZE;

  enable_eeprom_write();

  // CS low
  digitalWrite(9, LOW);

  // instruction
  SPI.transfer(EEPROM_WRITE);

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
}

byte read_eeprom_byte(uint16_t address) {
  // CS low
  digitalWrite(9, LOW);

  // read instruction
  SPI.transfer(EEPROM_READ);

  // MSB of address
  SPI.transfer(address >> 8);

  // LSB of address
  SPI.transfer(address & 255);

  // get value at address ("1" value doesn't matter, just have to send something to get a response)
  byte val = SPI.transfer(1);

  // CS back high to end action
  digitalWrite(9, HIGH);

  return val;
}

void enable_eeprom_write() {
  // CS low to take action
  digitalWrite(9, LOW);

  // read instruction
  SPI.transfer(EEPROM_WRITE_ENABLE);

  // CS high to end action
  digitalWrite(9, HIGH);
}
