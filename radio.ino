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

// the following clumps of variables control state for the available buttons.

// s1 = state of button
// s1_trigger = was the button pressed
// s1_latch = the button was pressed, now we play the associated sample to completion (even if button was released)
// bd1 = boomerang direction for this button (forward == 0, backward = 1)
// ls1 = LAST state of button
// ld1 = last debounce time

// all of the above applies to s2, s3, s4, etc

byte s1, s1_latch, bd1;
byte ls1 = HIGH;
unsigned long ld1 = 0;

byte s2, s2_latch, bd2;
byte ls2 = HIGH;
unsigned long ld2 = 0;

byte s3, s3_latch, bd3;
byte ls3 = HIGH;
unsigned long ld3 = 0;

byte s4, s4_latch, bd4;
byte ls4 = HIGH;
unsigned long ld4 = 0;

byte s5, s5_latch, bd5;
byte ls5 = HIGH;
unsigned long ld5 = 0;

byte s6, s6_latch, bd6;
byte ls6 = HIGH;
unsigned long ld6 = 0;

byte s7, s7_latch, bd7;
byte ls7 = HIGH;
unsigned long ld7 = 0;

byte s8, s8_latch, bd8;
byte ls8 = HIGH;
unsigned long ld8 = 0;

byte s9, s9_latch, bd9;
byte ls9 = HIGH;
unsigned long ld9 = 0;

byte s10, s10_latch, bd10;
byte ls10 = HIGH;
unsigned long ld10 = 0;

byte s11, s11_latch, bd11;
byte ls11 = HIGH;
unsigned long ld11 = 0;

// byte s12, s12_latch, bd12;
// byte ls12 = HIGH;
// unsigned long ld12 = 0;

byte sf;                // state of function button
byte lsf = HIGH;        // last state of function button
unsigned long ldf = 0;  // last debounce time of function button

// various modes of playback
// continuous = keep playing samples in a loop
// reverse = play backward
// boomerang = play forward then backward as one unit
// am = like a.m. radio, use amplitude modulation to make the sound
byte continuous, reverse, boomerang, am = 0;

// not audio "delay", but button press delay. If a button action happens, wait 50 ms to check the button again
const unsigned long debounceDelay = 50;

int sample_out_temp;  // used for gathering currently playing samples, then chopped down to 1 byte
byte sample_out;      // actual byte delivered to the DAC

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

const unsigned long dds_tune = 4294967296 / 8000;  // dds thing for sine playback for A.M. mode, don't worry about it

uint16_t index1, index2, index3, index4, index5, index6, index7, index8, index9, index10, index11, index12, index13, sine_index, window_start, window_end;
uint32_t accumulator1, accumulator2, accumulator3, accumulator4, accumulator5, accumulator6, accumulator7, accumulator8, accumulator9, accumulator10, accumulator11, accumulator12, accumulator13, sine_accumulator;

// Audio "delay"...play back stuff that happened already, in a repeating way, to make a pseudo echo effect
byte delay_buffer[1000] = { 0 };
byte delay_buffer_index, delay_active;

uint16_t sample_length;  // set every time we record, based on how long we record
byte recording;          // a boolean byte where 0 = false and 1 = true...are we currently recording?

byte initial_adcsra, initial_admux;  // used to store and restore values of ADC registers before and after recording
byte mp = 3;                         // max number of samples that can be playing at one time (based on speed of byte read from storage...too many and it'll choke!)

bool (*note1)() = NULL;
bool (*note2)() = NULL;
bool (*note3)() = NULL;

void set_ram_byte_mode() {
  digitalWrite(9, LOW);
  SPI.transfer(1);  // Instruction for setting RAM Mode is "1"

  // RAM Mode is set with a byte of data, from which only the two MSb are
  // important (this is "00" ("byte mode") but on top of a full byte (the rest are zeroes))
  SPI.transfer(0);
  digitalWrite(9, HIGH);
}

void set_ram_sequential_mode() {
  digitalWrite(9, LOW);
  SPI.transfer(1);  // Instruction for setting RAM Mode is "1"

  // RAM Mode is set with a byte of data, from which only the two MSb are
  // important (this is "01" ("sequential mode") but on top of a full byte (the rest are zeroes))
  SPI.transfer(64);
  digitalWrite(9, HIGH);
}

byte read_ram_byte(uint16_t address) {
  byte sample;

  digitalWrite(9, LOW);  // Pull CS of RAM low to begin interaction
  SPI.transfer(3);       // Instruction for READ is "3"
  SPI.transfer16(address);
  sample = SPI.transfer(0);  // send the byte of sample (passing in zero is inconsequential, it could be a 1 or whatever)
  digitalWrite(9, HIGH);     // CS goes high to end interaction

  return sample;
}

void write_ram_byte_sequential(byte sample) {

  // in sequential mode the RAM handles incrementing the address pointer, so just write the data!
  SPI.transfer(sample);

  // every time we write a byte, the sample length gets bigger
  sample_length++;
}

void begin_writing_ram_sequential() {
  digitalWrite(9, LOW);

  SPI.transfer(2);  // the "write" instruction for ram is "2"
  SPI.transfer16(0);

  // by recording we are reseting the sample length (it's destructive)
  // sample_length will be incremented with every successive write
  sample_length = 0;

  // this is a boolean "1"
  recording = 1;
}

void end_writing_ram_sequential() {
  digitalWrite(9, HIGH);  // Pull CS of RAM to HIGH to end this interaction

  recording = 0;  // boolean "0"
}

bool update1() {
  sample_out_temp += read_ram_byte(index1) - 127;

  accumulator1 += pitch;

  index1 = window_start + (accumulator1 >> (6));

  if (index1 > window_end) {
    index1 = window_start;
    accumulator1 = 0;
    return !(continuous && !s1);
  }

  return false;
}

bool update2() {
  sample_out_temp += read_ram_byte(index2) - 127;

  accumulator2 += minor_second;

  index2 = window_start + (accumulator2 >> (6));

  if (index2 > window_end || (continuous && s2)) {
    index2 = window_start;
    accumulator2 = 0;
    s2_latch = 0;

    return !(continuous && !s2);
  }

  return false;
}

bool update3() {
  sample_out_temp += read_ram_byte(index3) - 127;

  accumulator3 += major_second;

  index3 = window_start + (accumulator3 >> (6));

  if (index3 > window_end || (continuous && s3)) {
    index3 = window_start;
    accumulator3 = 0;
    s3_latch = 0;
    return !(continuous && !s3);
  }

  return false;
}

bool update4() {
  sample_out_temp += read_ram_byte(index4) - 127;

  accumulator4 += minor_third;

  index4 = window_start + (accumulator4 >> (6));

  if (index4 > window_end || (continuous && s4)) {
    index4 = window_start;
    accumulator4 = 0;
    s4_latch = 0;
    return !(continuous && !s4);
  }

  return false;
}

bool update5() {
  sample_out_temp += read_ram_byte(index5) - 127;

  accumulator5 += major_third;

  index5 = window_start + (accumulator5 >> (6));

  if (index5 > window_end || (continuous && s5)) {
    index5 = window_start;
    accumulator5 = 0;
    s5_latch = 0;
    return !(continuous && !s5);
  }

  return false;
}

bool update6() {
  sample_out_temp += read_ram_byte(index6) - 127;

  accumulator6 += perfect_fourth;

  index6 = window_start + (accumulator6 >> (6));

  if (index6 > window_end || (continuous && s6)) {
    index6 = window_start;
    accumulator6 = 0;
    s6_latch = 0;
    return !(continuous && !s6);
  }

  return false;
}

bool update7() {
  sample_out_temp += read_ram_byte(index7) - 127;

  accumulator7 += tritone;

  index7 = window_start + (accumulator7 >> (6));

  if (index7 > window_end || (continuous && s7)) {
    index7 = window_start;
    accumulator7 = 0;
    s7_latch = 0;
    return !(continuous && !s7);
  }

  return false;
}

bool update8() {
  sample_out_temp += read_ram_byte(index8) - 127;

  accumulator8 += perfect_fifth;

  index8 = window_start + (accumulator8 >> (6));

  if (index8 > window_end || (continuous && s8)) {
    index8 = window_start;
    accumulator8 = 0;
    s8_latch = 0;
    return !(continuous && !s8);
  }

  return false;
}

bool update9() {
  sample_out_temp += read_ram_byte(index9) - 127;

  accumulator9 += minor_sixth;

  index9 = window_start + (accumulator9 >> (6));

  if (index9 > window_end || (continuous && s9)) {
    index9 = window_start;
    accumulator9 = 0;
    s9_latch = 0;
    return !(continuous && !s9);
  }

  return false;
}

bool update10() {
  sample_out_temp += read_ram_byte(index10) - 127;

  accumulator10 += major_sixth;

  index10 = window_start + (accumulator10 >> (6));

  if (index10 > window_end || (continuous && s10)) {
    index10 = window_start;
    accumulator10 = 0;
    s10_latch = 0;
    return !(continuous && !s10);
  }

  return false;
}

bool update11() {
  sample_out_temp += read_ram_byte(index11) - 127;

  accumulator11 += major_seventh;

  index11 = window_start + (accumulator11 >> (6));

  if (index11 > window_end || (continuous && s11)) {
    index11 = window_start;
    accumulator11 = 0;
    s11_latch = 0;
    return !(continuous && !s11);
  }

  return false;
}

// bool update12() {
//   sample_out_temp += read_ram_byte(index12) - 127;

//   accumulator12 += major_seventh;

//   index12 = window_start + (accumulator12 >> (6));

//   if (index12 > window_end || (continuous && s12)) {
//     index12 = window_start;
//     accumulator12 = 0;
//     s12_latch = 0;
//     return !(continuous && !s12);
//   }

//   return false;
// }

void setup(void) {
  cli();  // disable Arduino interrupts to configure things without being interrupted

  // Use Arduino internal pullups to reduce necessary components. That means a button press will make a 0 and a button not-pressed will be 1!
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  // pinMode(A4, INPUT_PULLUP);

  pinMode(A5, OUTPUT);

  // CS pin for eeprom
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);  // bring it HIGH right away because LOW activates the chip (I don't make the rules)

  SPI.begin();  // Both the RAM and the DAC use SPI

  // Below: Set up the record/playback interrupt. It's annoying to remember all the register names.
  // Long story short, when the Arduino is on an interrupt will trigger 8000 times per second (holy cow)

  TCCR2A = 0;  // set entire TCCR2A register to 0
  TCCR2B = 0;  // same for TCCR2B
  TCNT2 = 0;   // initialize counter value to 0

  OCR2A = 249;              // set compare match register for 8khz increments (16,000,000 Hz / (8000 Hz * 8 bits per sample) - 1)
  TCCR2A |= (1 << WGM21);   // turn on CTC mode
  TCCR2B |= (1 << CS21);    // Set CS21 bit for 8 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt

  // capture initial values of ADC registers to restore when recording halts
  initial_adcsra = ADCSRA;
  initial_admux = ADMUX;

  sei();  // re-enable interrupts (let's do this)
}

ISR(TIMER2_COMPA_vect) {

  // "recording" boolean is set to 1 when recording starts
  // when recording ends, "recording" boolean is set to 0, so the rest of the code in
  // the interrupt will run instead
  if (recording) {

    // in sequential mode the RAM handles updating the address pointer after every write
    // so all we have to do is transfer another byte!
    write_ram_byte_sequential(ADCH);

    // early return from timer interrupt
    return;
  }

  // everything below here in the interrupt is playback code

  if (note1 != NULL) {
    if (note1()) note1 = NULL;
  }

  if (note2 != NULL) {
    if (note2()) note2 = NULL;
  }

  if (note3 != NULL) {
    if (note3()) note3 = NULL;
  }

  if (delay_active) sample_out_temp += delay_buffer[delay_buffer_index] - 127;

  // if (am) {
  //   // sample_out_temp = ((pgm_read_byte(&sine_table[sine_index]) - 127) * (sample1) / 255) << 1;
  // } else {
  // }

  sample_out_temp = (sample_out_temp >> 1) + 127;

  if (sample_out_temp > 255) {
    sample_out_temp -= (sample_out_temp - 255) << 1;
  }

  if (sample_out_temp < 0) {
    sample_out_temp += sample_out_temp * -2;
  }

  sample_out = sample_out_temp;

  delay_buffer[delay_buffer_index] = (sample_out >> 1);

  digitalWrite(10, LOW);  // CS pin for DAC LOW to start transaction

  // MCP4901 DAC "write" instruction is 16 bits. 15th bit (first bit in MSb) is always a 0.
  // Next three are config...look up them up in the datasheet.
  // NEXT 8 are the sample (4 bits on the 4 LSbs of byte 1, 4 bits on the MSbs of byte 2). Last 4 bits are "don't care" bits.
  // So, use SPI "transfer16" method to send it all as one 16 bit word.
  SPI.transfer16(0b0111000000000000 | (sample_out << 4));
  digitalWrite(10, HIGH);  // CS pin for DAC HIGH to end transaction

  // if (am) {
  //   sine_accumulator += 440 << 2;
  //   sine_index = (dds_tune * sine_accumulator) >> (32 - 8);
  // }

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
  // this is the smallest value possible with the prescaler config
  // we will actually be recording at 8000 samples per second
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRA |= (1 << ADATE);  // enable auto trigger
  // ADCSRA |= (1 << ADIE);   // enable interrupts
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  set_ram_sequential_mode();
  begin_writing_ram_sequential();

  digitalWrite(A5, HIGH);  // "recording" LED on

  sei();
}

void disable_record() {
  cli();

  // reset ADC register values to...defaults??
  ADCSRA = initial_adcsra;
  ADMUX = initial_admux;

  end_writing_ram_sequential();
  set_ram_byte_mode();

  TIMSK2 |= (1 << OCIE2A);  // enable playback

  digitalWrite(A5, LOW);  // "recording" LED off

  sei();
}

void assign_note(bool (*f)()) {
  if (note1 == f || note2 == f || note3 == f) return;
  
  if (note1 == NULL) {
    note1 = f;
  } else if (note2 == NULL) {
    note2 = f;
  } else if (note3 == NULL) {
    note3 = f;
  }
};

void loop(void) {
  byte d1 = digitalRead(2);
  byte d2 = digitalRead(3);
  byte d3 = digitalRead(4);
  byte d4 = digitalRead(5);
  byte d5 = digitalRead(6);
  byte d6 = digitalRead(7);
  byte d7 = digitalRead(0);
  byte d8 = digitalRead(1);
  byte d9 = digitalRead(A1);
  byte d10 = digitalRead(A2);
  byte d11 = digitalRead(A3);
  // byte d12 = digitalRead(A4);

  byte df = digitalRead(8);

  unsigned long time = millis();

  if (d1 == ls1) {
    ld1 = time;
  } else if (time - ld1 > debounceDelay) {
    s1 = d1;

    if (!s1) {
      if (!sf) {
        enable_record();
      } else {
        index1 = window_start;
        accumulator1 = 0;
        assign_note(update1);
      }
    } else {
      if (!sf) disable_record();
    }
  }

  if (d2 == ls2) {
    ld2 = time;
  } else if (time - ld2 > debounceDelay) {
    s2 = d2;

    if (!s2) {
      if (!sf) {
        delay_active = !delay_active;
        delay_buffer_index = 0;
      } else {
        index2 = window_start;
        accumulator2 = 0;
        assign_note(update2);
      }
    }
  }

  if (d3 == ls3) {
    ld3 = time;
  } else if (time - ld3 > debounceDelay) {
    s3 = d3;

    if (!s3) {
      if (!sf) {
        continuous = !continuous;
      } else {
        index3 = window_start;
        accumulator3 = 0;
        assign_note(update3);
      }
    }
  }

  if (d4 == ls4) {
    ld4 = time;
  } else if (time - ld4 > debounceDelay) {
    s4 = d4;

    if (!s4) {
      if (!sf) {
        reverse = !reverse;
        boomerang = 0;
      } else {
        index4 = window_start;
        accumulator4 = 0;
        assign_note(update4);
      }
    }
  }

  if (d5 == ls5) {
    ld5 = time;
  } else if (time - ld5 > debounceDelay) {
    s5 = d5;

    if (!s5) {
      if (!sf) {
        boomerang = !boomerang;
        reverse = 0;
      } else {
        index5 = window_start;
        accumulator5 = 0;
        assign_note(update5);
      }
    }
  }

  if (d6 == ls6) {
    ld6 = time;
  } else if (time - ld6 > debounceDelay) {
    s6 = d6;

    if (!s6) {
      if (!sf) {
        am = !am;
      } else {
        index6 = window_start;
        accumulator6 = 0;
        assign_note(update6);
      }
    }
  }

  if (d7 == ls7) {
    ld7 = time;
  } else if (time - ld7 > debounceDelay) {
    s7 = d7;

    if (!s7) {
      index7 = window_start;
      accumulator7 = 0;
      assign_note(update7);
    }
  }

  if (d8 == ls8) {
    ld8 = time;
  } else if (time - ld8 > debounceDelay) {
    s8 = d8;

    if (!s8) {
      index8 = window_start;
      accumulator8 = 0;
      assign_note(update8);
    }
  }

  if (d9 == ls9) {
    ld9 = time;
  } else if (time - ld9 > debounceDelay) {
    s9 = d9;

    if (!s9) {
      index9 = window_start;
      accumulator9 = 0;
      assign_note(update9);
    }
  }

  if (d10 == ls10) {
    ld10 = time;
  } else if (time - ld10 > debounceDelay) {
    s10 = d10;

    if (!s10) {
      index10 = window_start;
      accumulator10 = 0;
      assign_note(update10);
    }
  }

  if (d11 == ls11) {
    ld11 = time;
  } else if (time - ld11 > debounceDelay) {
    s11 = d11;

    if (!s11) {
      index11 = window_start;
      accumulator11 = 0;
      assign_note(update11);
    }
  }
  
  // if (d12 == ls12) {
  //   ld12 = time;
  // } else if (time - ld12 > debounceDelay) {
  //   s12 = d12;

  //   if (!s12) {
  //     index12 = window_start;
  //     accumulator12 = 0;
  //     assign_note(update12);
  //   }
  // }

  if (df == lsf) {
    ldf = time;
  } else if (time - ldf > debounceDelay) {
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
  ls9 = s9;
  ls10 = s10;
  ls11 = s11;
  // ls12 = s12;

  lsf = sf;

  // only take analogReads when not recording, it won't work when recording because of the adc register configuration
  if (!recording) {
    window_start = map(analogRead(A6), 0, 1024, 0, sample_length);
    window_end = map(analogRead(A7), 0, 1024, 0, sample_length);
  }
}
