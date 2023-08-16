#include <SPI.h>

#define LED_PIN A3

const byte sine_table[] PROGMEM = { 128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201,
                                    203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249,
                                    250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249,
                                    248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203, 201,
                                    198, 196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 128, 124,
                                    121, 118, 115, 112, 109, 106, 103, 100, 97, 93, 90, 88, 85, 82, 79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40, 37,
                                    35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2,
                                    2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62,
                                    65, 67, 70, 73, 76, 79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124 };

int8_t next_up = -1;
int8_t notes[3] = { -1, -1, -1 };
byte states[9] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };
uint16_t indexes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t envelope_start_points[3] = { 0, 0, 0 };
uint32_t accumulators[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
byte bds[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

byte windex_for_index_standard[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
byte windex_for_index_x4[8] = { 2, 2, 4, 4, 6, 6, 8, 8 };
byte *windex;

bool (*update_note)(int8_t, byte, byte) = NULL;

uint16_t sine_index;
uint32_t sine_accumulator;

unsigned long ld0, ld1, ld2, ld3, ld4, ld5, ld6, ld7, ld8, ld9, ld10, ld11 = 0;

byte sf = 1;            // state of function button
unsigned long ldf = 0;  // last debounce time of function button

// various modes of playback
// ring = keep playing samples in a loop
// reverse = play backward
// bounce = play forward then backward as one unit
// am = like a.m. radio, use amplitude modulation to make the sound
byte echo, ring, reverse, bounce, am, x4, envelope = 0;
byte echo_buffer[1500] = { 0 };
uint16_t echo_buffer_index;

// not audio "delay", but button press delay. If a button action happens, wait 50 ms to check the button again
const unsigned long debounceDelay = 50;

int sample_out_temp;  // used for gathering currently playing samples, then chopped down to 1 byte
byte sample_out;      // actual byte delivered to the DAC

byte pitch = 64;
float pitch_mod = 1.0;

float pitch_intervals[13] = {
  1.0,
  1.059463,  // minor second
  1.122462,  // major second
  1.189207,  // minor third
  1.259921,  // major third
  1.334840,  // perfect fourth
  1.414214,  // tritone
  1.498307,  // perfect fifth
  1.587401,  // minor sixth
  1.681793,  // major sixth
  1.781797,  // minor seventh
  1.887749,  // major seventh
  2.0        // octave
};

byte major_pitches[8] = {
  round(pitch * pitch_intervals[7] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[9] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[11] * 0.5 * pitch_mod),
  round(pitch * pitch_mod),
  round(pitch * pitch_intervals[2] * pitch_mod),
  round(pitch * pitch_intervals[4] * pitch_mod),
  round(pitch * pitch_intervals[5] * pitch_mod),
  round(pitch * pitch_intervals[7] * pitch_mod)
};

byte major_shifted_pitches[8] = {
  round(pitch * pitch_intervals[6] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[8] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[10] * 0.5 * pitch_mod),
  round(pitch * pitch_mod),
  round(pitch * pitch_intervals[1] * pitch_mod),
  round(pitch * pitch_intervals[3] * pitch_mod),
  round(pitch * pitch_intervals[5] * pitch_mod),
  round(pitch * pitch_intervals[6] * pitch_mod),
};

byte minor_pitches[8] = {
  round(pitch * pitch_intervals[7] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[10] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[11] * 0.5 * pitch_mod),
  round(pitch * pitch_mod),
  round(pitch * pitch_intervals[2] * pitch_mod),
  round(pitch * pitch_intervals[3] * pitch_mod),
  round(pitch * pitch_intervals[5] * pitch_mod),
  round(pitch * pitch_intervals[7] * pitch_mod)
};

byte minor_shifted_pitches[8] = {
  round(pitch * pitch_intervals[6] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[8] * 0.5 * pitch_mod),
  round(pitch * pitch_intervals[9] * 0.5 * pitch_mod),
  round(pitch * pitch_mod),
  round(pitch * pitch_intervals[1] * pitch_mod),
  round(pitch * pitch_intervals[4] * pitch_mod),
  round(pitch * pitch_intervals[5] * pitch_mod),
  round(pitch * pitch_intervals[6] * pitch_mod),
};

/**
  major: whole, whole, half, whole, whole, whole, half

  minor:
   - natural: whole, half, whole, whole, half, whole, whole
   - harmonic: whole, half, whole, whole, half, augmented second (?), half <--
   - melodic: whole, half, whole, whole, whole, whole, half
*/

byte x4_pitches[8] = {
  pitch,
  round(pitch * 1.498307),
  pitch,
  round(pitch * 1.498307),
  pitch,
  round(pitch * 1.498307),
  pitch,
  round(pitch * 1.498307)
};

byte *pitches;
byte major_or_minor;

const unsigned long dds_tune = 4294967296 / 8000;  // dds thing for sine playback for A.M. mode, don't worry about it

uint16_t windows[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t sample_length;  // set every time we record, based on how long we record
byte recording;          // a boolean byte where 0 = false and 1 = true...are we currently recording?

byte initial_adcsra, initial_admux;  // used to store and restore values of ADC registers before and after recording

void recalculate_pitches() {
  major_pitches[0] = pitch * pitch_intervals[0] * pitch_mod;
  major_pitches[1] = pitch * pitch_intervals[2] * pitch_mod;
  major_pitches[2] = pitch * pitch_intervals[4] * pitch_mod;
  major_pitches[3] = pitch * pitch_intervals[5] * pitch_mod;
  major_pitches[4] = pitch * pitch_intervals[7] * pitch_mod;
  major_pitches[5] = pitch * pitch_intervals[9] * pitch_mod;
  major_pitches[6] = pitch * pitch_intervals[11] * pitch_mod;
  major_pitches[7] = pitch * pitch_intervals[12] * pitch_mod;

  major_shifted_pitches[0] = pitch * pitch_intervals[0] * pitch_mod;
  major_shifted_pitches[1] = pitch * pitch_intervals[1] * pitch_mod;
  major_shifted_pitches[2] = pitch * pitch_intervals[3] * pitch_mod;
  major_shifted_pitches[3] = pitch * pitch_intervals[5] * pitch_mod;
  major_shifted_pitches[4] = pitch * pitch_intervals[6] * pitch_mod;
  major_shifted_pitches[5] = pitch * pitch_intervals[8] * pitch_mod;
  major_shifted_pitches[6] = pitch * pitch_intervals[10] * pitch_mod;
  major_shifted_pitches[7] = pitch * pitch_intervals[12] * pitch_mod;

  minor_pitches[0] = pitch * pitch_intervals[0] * pitch_mod;
  minor_pitches[1] = pitch * pitch_intervals[2] * pitch_mod;
  minor_pitches[2] = pitch * pitch_intervals[3] * pitch_mod;
  minor_pitches[3] = pitch * pitch_intervals[5] * pitch_mod;
  minor_pitches[4] = pitch * pitch_intervals[7] * pitch_mod;
  minor_pitches[5] = pitch * pitch_intervals[10] * pitch_mod;
  minor_pitches[6] = pitch * pitch_intervals[11] * pitch_mod;
  minor_pitches[7] = pitch * pitch_intervals[12] * pitch_mod;

  minor_shifted_pitches[0] = pitch * pitch_intervals[0] * pitch_mod;
  minor_shifted_pitches[1] = pitch * pitch_intervals[1] * pitch_mod;
  minor_shifted_pitches[2] = pitch * pitch_intervals[4] * pitch_mod;
  minor_shifted_pitches[3] = pitch * pitch_intervals[5] * pitch_mod;
  minor_shifted_pitches[4] = pitch * pitch_intervals[6] * pitch_mod;
  minor_shifted_pitches[5] = pitch * pitch_intervals[8] * pitch_mod;
  minor_shifted_pitches[6] = pitch * pitch_intervals[9] * pitch_mod;
  minor_shifted_pitches[7] = pitch * pitch_intervals[12] * pitch_mod;

  x4_pitches[0] = pitch * pitch_intervals[0] * pitch_mod;
  x4_pitches[1] = pitch * pitch_intervals[7] * pitch_mod;
  x4_pitches[2] = pitch * pitch_intervals[0] * pitch_mod;
  x4_pitches[3] = pitch * pitch_intervals[7] * pitch_mod;
  x4_pitches[4] = pitch * pitch_intervals[0] * pitch_mod;
  x4_pitches[5] = pitch * pitch_intervals[7] * pitch_mod;
  x4_pitches[6] = pitch * pitch_intervals[0] * pitch_mod;
  x4_pitches[7] = pitch * pitch_intervals[7] * pitch_mod;
}

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
}

void end_writing_ram_sequential() {
  digitalWrite(9, HIGH);  // Pull CS of RAM to HIGH to end this interaction
}

void update_note_reverse(int8_t index, byte note_index, byte windex) {
  accumulators[index] += pitches[index];
  indexes[index] = windows[windex + 1] - (accumulators[index] >> 6);
  sample_out_temp += read_ram_byte(indexes[index]) - 127;

  if (indexes[index] < windows[windex]) {
    accumulators[index] = 0;

    if (!(ring && !states[index])) {
      if (next_up != -1) {
        notes[note_index] = next_up;
        next_up = -1;
      } else {
        notes[note_index] = -1;
      }
    }
  }
}

void update_note_bounce(int8_t index, byte note_index, byte windex) {
  accumulators[index] += pitches[index];

  if (bds[index]) {
    indexes[index] = windows[windex + 1] - (accumulators[index] >> 6);
    sample_out_temp += read_ram_byte(indexes[index]) - 127;

    if (indexes[index] < windows[windex]) {
      accumulators[index] = 0;
      bds[index] = 0;

      if (!(ring && !states[index])) {
        if (next_up != -1) {
          notes[note_index] = next_up;
          next_up = -1;
        } else {
          notes[note_index] = -1;
        }
      }
    }
  } else {
    indexes[index] = windows[windex] + (accumulators[index] >> 6);
    sample_out_temp += read_ram_byte(indexes[index]) - 127;

    if (indexes[index] > windows[windex + 1]) {
      accumulators[index] = 0;
      bds[index] = 1;
    }
  }
}

void update_note_basic(int8_t index, byte note_index, byte windex) {
  accumulators[index] += pitches[index];
  indexes[index] = windows[windex] + (accumulators[index] >> 6);
  sample_out_temp += read_ram_byte(indexes[index]) - 127;

  if (indexes[index] > windows[windex + 1]) {
    accumulators[index] = 0;

    if (!(ring && !states[index])) {
      if (next_up != -1) {
        notes[note_index] = next_up;
        next_up = -1;
      } else {
        notes[note_index] = -1;
      }
    }
  }
}

void update_note_envelope(int8_t index, byte note_index, byte windex) {
  accumulators[index] += pitches[index];
  indexes[index] = envelope_start_points[note_index] + (accumulators[index] >> 6);
  sample_out_temp += read_ram_byte(indexes[index]) - 127;

  if (states[index]) {
    if (indexes[index] > sample_length) {
      envelope_start_points[note_index] = 0;

      if (next_up != -1) {
        notes[note_index] = next_up;
        next_up = -1;
      } else {
        notes[note_index] = -1;
      }
    }
  } else if (indexes[index] > windows[windex + 1]) {
    envelope_start_points[note_index] = windows[windex];
    accumulators[index] = 0;
  }
}

void setup(void) {
  cli();  // disable Arduino interrupts to configure things without being interrupted

  update_note = update_note_basic;
  pitches = major_pitches;
  windex = windex_for_index_standard;

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

  pinMode(LED_PIN, OUTPUT);

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

  led_flash(1);
}

byte passthrough, clipping, clipping_counter, timed_out = 0;

ISR(TIMER2_COMPA_vect) {
  sample_out_temp = 0;

  // "recording" boolean is set to 1 when recording starts
  // when recording ends, "recording" boolean is set to 0, so the rest of the code in
  // the interrupt will run instead
  if (recording) {
    byte sample = ADCH;

    if (!passthrough) {
      // in sequential mode the RAM handles updating the address pointer after every write
      // so all we have to do is transfer another byte!
      write_ram_byte_sequential(sample);

      if (sample_length > 40000) {
        timed_out = 1;
        disable_record();
      }
    } else {
      digitalWrite(10, LOW);  // CS pin for DAC LOW to start transaction
      // MCP4901 DAC "write" instruction is 16 bits. 15th bit (first bit in MSb) is always a 0.
      // Next three are config...look up them up in the datasheet.
      // NEXT 8 are the sample (4 bits on the 4 LSbs of byte 1, 4 bits on the MSbs of byte 2). Last 4 bits are "don't care" bits.
      // So, use SPI "transfer16" method to send it all as one 16 bit word.
      SPI.transfer16(0b0111000000000000 | (sample << 4));
      digitalWrite(10, HIGH);  // CS pin for DAC HIGH to end transaction
    }

    if (clipping_counter > 0) {
      clipping_counter--;
    } else if (sample < 50 || sample > 204) {
      clipping = 1;
      clipping_counter = 500;
      digitalWrite(LED_PIN, LOW);
    } else if (clipping) {
      clipping = 0;
      digitalWrite(LED_PIN, HIGH);
    }

    // early return from timer interrupt
    return;
  }

  // params: button index (from 0 to 8), note index (from 0 to 2), window index (from 0 to 9)
  if (notes[0] != -1) update_note(notes[0], 0, windex[notes[0]]);
  if (notes[1] != -1) update_note(notes[1], 1, windex[notes[1]]);
  if (notes[2] != -1) update_note(notes[2], 2, windex[notes[2]]);

  if (echo) sample_out_temp += echo_buffer[echo_buffer_index] - 127;

  if (am) {
    sample_out_temp = ((int)((pgm_read_byte(&sine_table[sine_index]) - 127) * (sample_out_temp / 127.0))) + 127;
    sine_accumulator += 220 << 2;
    sine_index = (dds_tune * sine_accumulator) >> 24;
  }

  sample_out_temp = (sample_out_temp >> 1) + 127;

  if (sample_out_temp > 255) {
    sample_out_temp -= (sample_out_temp - 255) << 1;
  }

  if (sample_out_temp < 0) {
    sample_out_temp += sample_out_temp * -2;
  }

  sample_out = sample_out_temp;

  if (echo) {
    echo_buffer[echo_buffer_index] = (sample_out >> 1);
    echo_buffer_index++;
    if (echo_buffer_index == 1500) echo_buffer_index = 0;
  }

  digitalWrite(10, LOW);  // CS pin for DAC LOW to start transaction
  // MCP4901 DAC "write" instruction is 16 bits. 15th bit (first bit in MSb) is always a 0.
  // Next three are config...look up them up in the datasheet.
  // NEXT 8 are the sample (4 bits on the 4 LSbs of byte 1, 4 bits on the MSbs of byte 2). Last 4 bits are "don't care" bits.
  // So, use SPI "transfer16" method to send it all as one 16 bit word.
  SPI.transfer16(0b0111000000000000 | (sample_out << 4));
  digitalWrite(10, HIGH);  // CS pin for DAC HIGH to end transaction
}

void enable_record() {
  cli();  // disable interrupt for playback timer

  // TIMSK2 |= (0 << OCIE2A);  // disable playback

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

  if (!passthrough) {
    set_ram_sequential_mode();
    begin_writing_ram_sequential();
  }

  recording = 1;

  digitalWrite(LED_PIN, HIGH);  // "recording" LED on

  sei();
}

void disable_record() {
  cli();

  // reset ADC register values to...defaults??
  ADCSRA = initial_adcsra;
  ADMUX = initial_admux;

  if (!passthrough) {
    end_writing_ram_sequential();
    set_ram_byte_mode();
  }

  recording = 0;

  // TIMSK2 |= (1 << OCIE2A);  // enable playback

  if (!passthrough) {
    pitch_mod = 1.0;
    recalculate_pitches();
  }

  digitalWrite(LED_PIN, LOW);  // "recording" LED off

  sei();

  led_flash(2);
}

void assign_note(int8_t note_index) {
  if (notes[0] == note_index || notes[1] == note_index || notes[2] == note_index) return;

  if (notes[0] == -1) {
    notes[0] = note_index;
  } else if (notes[1] == -1) {
    notes[1] = note_index;
  } else if (notes[2] == -1 && !ring && !envelope) {
    notes[2] = note_index;
  } else {
    next_up = note_index;
  }
};

void led_flash(byte times) {
  byte i = 0;

  for (i; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(100);
  }
}

void loop(void) {
  byte p = PIND;                // digital pins 0-7 register
  byte df = PINB & 0b00000001;  // digital pins 8-? register (don't care, only need 8)
  byte pa = PINC;               // analog pins register

  byte d0 = p & 0b00000100;
  byte d1 = p & 0b00001000;
  byte d2 = p & 0b00010000;
  byte d3 = p & 0b00100000;
  byte d4 = p & 0b01000000;
  byte d5 = p & 0b10000000;
  byte d6 = p & 0b00000001;
  byte d7 = p & 0b00000010;
  byte d8 = pa & 0b00000010;

  unsigned long time = millis();

  if (d0 == states[0]) {
    ld0 = time;
  } else if (time - ld0 > debounceDelay) {
    states[0] = d0;

    if (!states[0]) {
      if (!sf) {
        if (!states[7]) {
          windows[2] = map(analogRead(A4), 0, 1024, 100, sample_length);
          windows[3] = pitch * (map(analogRead(A5), 0, 1024, windows[2], sample_length) / pitch);
          led_flash(1);
        } else {
          enable_record();
        }
      } else {
        accumulators[0] = 0;
        assign_note(0);
      }
    } else {
      if (!sf) {
        if (!timed_out) {
          disable_record();
        } else {
          timed_out = 0;
        }
      }
    }
  }

  if (d1 == states[1]) {
    ld1 = time;
  } else if (time - ld1 > debounceDelay) {
    states[1] = d1;

    if (!states[1]) {
      if (!sf) {
        if (!states[7]) {
          windows[4] = map(analogRead(A4), 0, 1024, 100, sample_length);
          windows[5] = pitch * (map(analogRead(A5), 0, 1024, windows[4], sample_length) / pitch);
          led_flash(2);
        } else {
          envelope = !envelope;

          if (envelope) {
            ring = 0;
            bounce = 0;
            reverse = 0;
            update_note = update_note_envelope;
          } else {
            update_note = update_note_basic;
          }
        }
      } else {
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
        if (!states[7]) {
          windows[6] = map(analogRead(A4), 0, 1024, 100, sample_length);
          windows[7] = pitch * (map(analogRead(A5), 0, 1024, windows[6], sample_length) / pitch);
          led_flash(3);
        } else {
          ring = !ring;

          if (ring && envelope) {
            envelope = 0;
            update_note = update_note_basic;
          }
        }
      } else {
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
        if (!states[7]) {
          windows[8] = map(analogRead(A4), 0, 1024, 100, sample_length);
          windows[9] = pitch * (map(analogRead(A5), 0, 1024, windows[8], sample_length) / pitch);
          led_flash(4);
        } else {
          reverse = !reverse;

          if (reverse) {
            bounce = 0;
            envelope = 0;
            update_note = update_note_reverse;
          } else {
            update_note = update_note_basic;
          }
        }
      } else {
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
        if (!states[7]) {
          if (echo) am = 0;
          echo = !echo;
          echo_buffer_index = 0;
        } else {
          bounce = !bounce;

          if (bounce) {
            reverse = 0;
            envelope = 0;
            update_note = update_note_bounce;
          } else {
            update_note = update_note_basic;
          }
        }
      } else {
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
        if (!states[7]) {
          pitch_mod *= 0.5;
          recalculate_pitches();
        } else {
          am = !am;
          echo = 0;
        }
      } else {
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
      if (!sf) {
        if (!states[7]) {
          pitch_mod *= 2;
          recalculate_pitches();
        } else {
          x4 = !x4;

          if (x4) {
            pitches = x4_pitches;
            windex = windex_for_index_x4;
          } else {
            pitches = major_or_minor ? minor_pitches : major_pitches;
            windex = windex_for_index_standard;
          }

          update_note = update_note_basic;
        }
      } else {
        accumulators[6] = 0;
        assign_note(6);
      }
    }
  }

  if (d7 == states[7]) {
    ld7 = time;
  } else if (time - ld7 > debounceDelay) {
    states[7] = d7;

    if (!states[7]) {
      if (sf) {  // if NOT function key (active low, remember)
        accumulators[7] = 0;
        assign_note(7);
      }
    }
  }

  if (d8 == states[8]) {
    ld8 = time;
  } else if (time - ld8 > debounceDelay) {
    states[8] = d8;

    if (!states[8]) {
      if (!sf) {
        if (!states[7]) {
          if (passthrough) {
            disable_record();
            passthrough = 0;
          } else {
            passthrough = 1;
            enable_record();
          }
        } else {
          major_or_minor = !major_or_minor;
          pitches = major_or_minor ? minor_pitches : major_pitches;
        }
      } else {
        pitches = major_or_minor ? minor_shifted_pitches : major_shifted_pitches;
      }
    } else {
      pitches = major_or_minor ? minor_pitches : major_pitches;
    }
  }

  if (df == sf) {
    ldf = time;
  } else if (time - ldf > debounceDelay) {
    sf = df;

    if (sf) {
      if (!states[0]) disable_record();
    }
  }

  // only take analogReads when not recording, it won't work when recording because of the adc register configuration
  if (!recording && time % 500 == 0) {
    windows[0] = map(analogRead(A4), 0, 1024, 100, sample_length);
    windows[1] = pitch * (map(analogRead(A5), 0, 1024, windows[0], sample_length) / pitch);
  }
}
