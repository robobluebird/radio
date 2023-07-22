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

int8_t notes[3]           = { -1, -1, -1 };
byte states[12]           = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
uint16_t indexes[12]      = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t accumulators[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte bds[12]              = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t sine_index;
uint32_t sine_accumulator;

unsigned long ld0, ld1, ld2, ld3, ld4, ld5, ld6, ld7, ld8, ld9, ld10, ld11 = 0;

byte sf = 1;            // state of function button
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
// byte minor_second = round(pitch * 1.059463);
// byte major_second = round(pitch * 1.122462);
// byte minor_third = round(pitch * 1.189207);
// byte major_third = round(pitch * 1.259921);
// byte perfect_fourth = round(pitch * 1.334840);
// byte tritone = round(pitch * 1.414214);
// byte perfect_fifth = round(pitch * 1.498307);
// byte minor_sixth = round(pitch * 1.587401);
// byte major_sixth = round(pitch * 1.681793);
// byte minor_seventh = round(pitch * 1.781797);
// byte major_seventh = round(pitch * 1.887749);
// byte octave = round(pitch * 2.0);

byte pitches[13] = {
  pitch,
  round(pitch * 1.059463),
  round(pitch * 1.122462),
  round(pitch * 1.189207),
  round(pitch * 1.259921),
  round(pitch * 1.334840),
  round(pitch * 1.414214),
  round(pitch * 1.498307),
  round(pitch * 1.587401),
  round(pitch * 1.681793),
  round(pitch * 1.781797),
  round(pitch * 1.887749),
  round(pitch * 2.0)
};

const unsigned long dds_tune = 4294967296 / 8000;  // dds thing for sine playback for A.M. mode, don't worry about it

uint16_t w1, e1, w2, e2, w3, e3, w4, e4, window_start, window_end;

// Audio "delay"...play back stuff that happened already, in a repeating way, to make a pseudo echo effect
byte delay_buffer[1000] = { 0 };
byte delay_buffer_index, delay_active;

uint16_t sample_length;  // set every time we record, based on how long we record
byte recording;          // a boolean byte where 0 = false and 1 = true...are we currently recording?

byte initial_adcsra, initial_admux;  // used to store and restore values of ADC registers before and after recording
byte mp = 3;                         // max number of samples that can be playing at one time (based on speed of byte read from storage...too many and it'll choke!)

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

bool update_note(int8_t index) {
  sample_out_temp += read_ram_byte(indexes[index]) - 127;

  accumulators[index] += pitches[index];

  if (!reverse && (!boomerang || !bds[index])) {
    indexes[index] = window_start + (accumulators[index] >> 6);

    if (indexes[index] > window_end) {
      indexes[index] = window_start;
      accumulators[index] = 0;

      if (boomerang) {
        bds[index] = 1;
        return false;
      }

      return !(continuous && !states[index]);
    }
  } else {
    indexes[index] = window_end - (accumulators[index] >> 6);

    if (indexes[index] < window_start) {
      indexes[index] = window_end;
      accumulators[index] = 0;

      if (boomerang) bds[index] = 0;

      return !(continuous && !states[index]);
    }
  }

  return false;
}

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
  pinMode(A4, INPUT_PULLUP);

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

  if (notes[0] != -1) {
    if (update_note(notes[0])) notes[0] = -1;
  }

  if (notes[1] != -1) {
    if (update_note(notes[1])) notes[1] = -1;
  }

  if (notes[2] != -1) {
    if (update_note(notes[2])) notes[2] = -1;
  }

  if (delay_active) sample_out_temp += delay_buffer[delay_buffer_index] - 127;

  if (am) {
    sample_out_temp = ((pgm_read_byte(&sine_table[sine_index]) - 127) * ((sample_out_temp / 3) / 255)) << 1;
  }

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

  if (am) {
    sine_accumulator += 440 << 2;
    sine_index = (dds_tune * sine_accumulator) >> (32 - 8);
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

void assign_note(int8_t note_index) {
  if (notes[0] == note_index || notes[1] == note_index || notes[2] == note_index) return;

  if (notes[0] == -1) {
    notes[0] = note_index;
  } else if (notes[1] == -1) {
    notes[1] = note_index;
  } else if (notes[2] == -1) {
    notes[2] = note_index;
  }
};

void loop(void) {
  byte d0 = digitalRead(2);
  byte d1 = digitalRead(3);
  byte d2 = digitalRead(4);
  byte d3 = digitalRead(5);
  byte d4 = digitalRead(6);
  byte d5 = digitalRead(7);
  byte d6 = digitalRead(0);
  byte d7 = digitalRead(1);
  byte d8 = digitalRead(A1);
  byte d9 = digitalRead(A4);
  byte d10 = digitalRead(A2);
  byte d11 = digitalRead(A3);

  byte df = digitalRead(8);

  unsigned long time = millis();

  if (d0 == states[0]) {
    ld0 = time;
  } else if (time - ld0 > debounceDelay) {
    states[0] = d0;

    if (!states[0]) {
      if (!sf) {
        enable_record();
      } else {
        indexes[0] = reverse ? window_end : window_start;
        accumulators[0] = 0;
        assign_note(0);
      }
    } else {
      if (!sf) disable_record();
    }
  }

  if (d1 == states[1]) {
    ld1 = time;
  } else if (time - ld1 > debounceDelay) {
    states[1] = d1;

    if (!states[1]) {
      if (!sf) {
        delay_active = !delay_active;
        delay_buffer_index = 0;
      } else {
        indexes[1] = reverse ? window_end : window_start;
        accumulators[1] = 0;
        assign_note(1);
      }
    }
  }

  if (d2 == states[2]) {
    ld2 = time;
  } else if (time - ld2 > debounceDelay) {
    states[2] = d2;

    if (!states[2]) {
      if (!sf) {
        continuous = !continuous;
      } else {
        indexes[2] = reverse ? window_end : window_start;
        accumulators[2] = 0;
        assign_note(2);
      }
    }
  }

  if (d3 == states[3]) {
    ld3 = time;
  } else if (time - ld3 > debounceDelay) {
    states[3] = d3;

    if (!states[3]) {
      if (!sf) {
        reverse = !reverse;
        boomerang = 0;
      } else {
        indexes[3] = reverse ? window_end : window_start;
        accumulators[3] = 0;
        assign_note(3);
      }
    }
  }

  if (d4 == states[4]) {
    ld4 = time;
  } else if (time - ld4 > debounceDelay) {
    states[4] = d4;

    if (!states[4]) {
      if (!sf) {
        boomerang = !boomerang;
        reverse = 0;
      } else {
        indexes[4] = reverse ? window_end : window_start;
        accumulators[4] = 0;
        assign_note(4);
      }
    }
  }

  if (d5 == states[5]) {
    ld5 = time;
  } else if (time - ld5 > debounceDelay) {
    states[5] = d5;

    if (!states[5]) {
      if (!sf) {
        am = !am;
        sine_index = 0;
        sine_accumulator = 0;
      } else {
        indexes[5] = reverse ? window_end : window_start;
        accumulators[5] = 0;
        assign_note(5);
      }
    }
  }

  if (d6 == states[6]) {
    ld6 = time;
  } else if (time - ld6 > debounceDelay) {
    states[6] = d6;

    if (!states[6]) {
      indexes[6] = reverse ? window_end : window_start;
      accumulators[6] = 0;
      assign_note(6);
    }
  }

  if (d7 == states[7]) {
    ld7 = time;
  } else if (time - ld7 > debounceDelay) {
    states[7] = d7;

    if (!states[7]) {
      indexes[7] = reverse ? window_end : window_start;
      accumulators[7] = 0;
      assign_note(7);
    }
  }

  if (d8 == states[8]) {
    ld8 = time;
  } else if (time - ld8 > debounceDelay) {
    states[8] = d8;

    if (!states[8]) {
      indexes[8] = reverse ? window_end : window_start;
      accumulators[8] = 0;
      assign_note(8);
    }
  }

  if (d9 == states[9]) {
    ld9 = time;
  } else if (time - ld9 > debounceDelay) {
    states[9] = d9;

    if (!states[9]) {
      indexes[9] = reverse ? window_end : window_start;
      accumulators[9] = 0;
      assign_note(9);
    }
  }

  if (d10 == states[10]) {
    ld10 = time;
  } else if (time - ld10 > debounceDelay) {
    states[10] = d10;

    if (!states[10]) {
      indexes[10] = reverse ? window_end : window_start;
      accumulators[10] = 0;
      assign_note(10);
    }
  }

  if (d11 == states[11]) {
    ld11 = time;
  } else if (time - ld11 > debounceDelay) {
    states[11] = d11;

    if (!states[11]) {
      indexes[11] = reverse ? window_end : window_start;
      accumulators[11] = 0;
      assign_note(11);
    }
  }

  if (df == sf) {
    ldf = time;
  } else if (time - ldf > debounceDelay) {
    sf = df;
  }

  // only take analogReads when not recording, it won't work when recording because of the adc register configuration
  if (!recording) {
    window_start = map(analogRead(A6), 0, 1024, 100, sample_length);
    window_end = map(analogRead(A7), 0, 1024, window_start, sample_length);
  }
}
