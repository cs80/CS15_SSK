#include <MIDI.h>               // MIDI Library 4.1
#include <midi_Defs.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>

#include <interrupts.h>
#include <pgmspace.h>           // For tables in ROM
#include <SPI.h>                // used to communicate with the DAC
#include <DAC_MCP49xx.h>        // DAC lib


/* Old Crow's Yamaha SSK YM24800 Replacement
   Warning: This is my first Arduino IDE-based project. ;)  I am
   not really a C programmer.  I am a good assembly programmer,
   and I used to write stuff in Pascal 25 years ago. So I at
   least have a coder's mindset even if it took a week to knock all
   the rust loose. Oddly, there wasn't much in the way of useful Arduino
   MIDI coding examples out there, so most of what you see here is
   transcribed from my old Zilog Z8 code methods for MIDI. I used Trevor
   Page's note buffering scheme, though his original is in (14-bit) PIC
   assembly that I had to transcribe it to C.
   --Crow

   Yamaha SSK Key Assigner with MIDI by Scott Rider, aka The Old Crow
   cc-by-sa ** OPEN SOURCE: FREE FOR NON-COMMERCIAL USE **
   
   Acknowledgments:
   Jurgen Haible (RIP) -- I will always pay it forward.
   Trevor Page -- the RX303a PIC code for your MIDI to CV/gate is awesome!
   Various Arduino and C folks -- thanks!   

   Revision: 1.07.13
   LAST MODIFIED 23-APR-2014 07:08
   (Code rewrite to use common buffer and note table schemes)
   
   Emulates the 37-note keyboard scanner and V/Hz DAC of the Yamaha YM24800
   chip as used in the Yamaha CS15, etc.  It also sends and receives MIDI
   currently on MIDI channel 1.

   This program scans a 37-note matrix-wired keybed as six groups of six with
   an extra note (low C).  Each key found will transmit a MIDI note as
   well as load the DACs with octave and pitch data for the most recently
   pressed key and enable the GATE signal. As keys are released, MIDI note
   off codes (note on w/velocity=0) are sent, but the GATE is only disabled
   once all keys are released--physical keys and received MIDI notes.
   
   The circuit uses a 74HC595 as a simple output port as part of the keybed
   scanner.  It also uses a dual 12-bit SPI DAC wired such that the externally-
   provided pitchbend voltage sets the Vref of the first DAC, the DAC codes
   of which then select the desired octave.  Only four octave codes are used
   for the local keyboard: 512, 1024, 2048 and 4095.  Codes 128 and 256 are
   also used for MIDI in to extend the usable range. The output of the octave
   DAC is connected to be the Vref of the second DAC, which becomes the note DAC
   as it is loaded with 1 of 12 semitone values as determined by the key scanner.
   This chaining of DACs is necessary to maintain the exponential relationship of
   the note intervals in a V/Hz (linear) response voltage control scheme without
   loss of DAC resolution. Linear (V/Hz) response is used by Yamaha in most of
   their vintage analog gear.
   
   To do: Use EEROM to store microtuning tables, channel numbers, etc.
*/

#define  bGrab  pgm_read_byte_near // Define shorter names for these progmem calls
#define  wGrab  pgm_read_word_near //  "   "

#define DACCS 10 // Arduino pin 10 / 328P pin 16 DACCS (chip select)
#define LDACS 7  // Arduino pin 7 / 328P pin 13 LDAC (latch enable)

//Some literals for indexing the noteDAC[] table
#define noteCS 0  // C#
#define noteD  1  // D
#define noteDS 2  // D#
#define noteE  3  // E
#define noteF  4  // F
#define noteFS 5  // F#
#define noteG  6  // G
#define noteGS 7  // G#
#define noteA  8  // A
#define noteAS 9  // A#
#define noteB  10 // B
#define noteC  11 // C

//Some literals for indexing the octDAC[] table
#define oct0  0
#define oct1  1
#define oct2  2
#define oct3  3
#define oct4  4
#define oct5  5

// Index values for each keybed group
#define GRP1  1
#define GRP2  7
#define GRP3  13
#define GRP4  19
#define GRP5  25
#define GRP6  31

// Tell DAC library which part and pins to use
DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, DACCS, LDACS); // DAC initialize
/*
  dac.setBuffer(bool) true to enable external Vref buffers, off by default
  dac.setGain(int) set DAC gain 1x or 2x, use value of 1 (default) or 2
  dac.output2(ushort val1, ushort val2) to send values to dac
*/

//
// Pin Definitions
// Matrix keybed rows
const int rowCsG = 2; // 328P pin 4
const int rowDGs = 3; // 328P pin 5
const int rowDsA = 4; // 328P pin 6
const int rowEAs = 5; // 328P pin 11
const int rowFB  = 6; // 328P pin 12
const int rowFsC = 9; // 328P pin 15
const int rowCL  = 15; // 328P pin 27 *shared with data595* (input)

// Gate signal to trigger CS15 hardware
const int gate = 8; // 328P pin 14

// The 74HC595 uses three pins
const int clock595 = 16; // 328P pin 25
const int latch595 = 17; // 328P pin 26
const int data595 =  15; // 328P pin 27 *shared with rowCL* (output)

// Lookup tables in program flash area

// Map the physical keyboard keys to their MIDI note numbers.
// Lowest value is 48 which corresponds to C3 MIDI note.
PROGMEM prog_uint8_t midiKeyVal[] = { 48,                   // groupCL  = C3
                                    49, 55, 61, 67, 73, 79, // groupCsG = C#3, G3, C#4, G4, C#5, G5
                                    50, 56, 62, 68, 74, 80, // groupDGs = D3, G#3, D4, G#4, D5, G#5
                                    51, 57, 63, 69, 75, 81, // groupDsA = D#3, A3, D#4, A4, D#5, A5
                                    52, 58, 64, 70, 76, 82, // groupEAs = E3, A#3, E4, A#4, E5, A#5
                                    53, 59, 65, 71, 77, 83, // groupFB  = F3, B3, F4, B4, F5, B5
                                    54, 60, 66, 72, 78, 84 }; // groupFsC = F#3, C4, F#4, C5, F#5, C6
// Index of octaves
PROGMEM prog_uint16_t octDAC[] = { 128, 256, 512, 1024, 2048, 4095 };

// noteDAC stores the 12-bit DAC code values for the 12 semitones.
// This list is for equal-tempered (12th root of 2) pitches.
// Example: note C uses the highest code available = 4095. Note B = 4095 / 12√2 = 3865, etc.
PROGMEM prog_uint16_t noteDAC[] = { 2169, 2298, 2435, 2580, 2733, 2896,   // C#, D, D#, E, F, F#
                                    3068, 3250, 3443, 3648, 3865, 4095 }; // G, G#, A, A#, B, C


// noteMap is every MIDI note code from 0 to 127 arranged a two-term byte array lookup table so as to map
// incoming MIDI note values to the useable range of the CS15 and provide the relevant octave and note DAC
// codes for a given note.  Format: octDAC[] offset, noteDAC[] offset
// Also used by physical key assigner to look up octave and note values.
// The octave alignment is skewed somewhat by that 37th physical key, being key CL = code 48
PROGMEM prog_uint8_t noteMap[] = { oct0, noteC, // lowest MIDI note (MIDI key code 00), remap notes below code 48 up 3 octaves
                                 oct1, noteCS, oct1, noteD,  oct1, noteDS, oct1, noteE,  oct1, noteF, oct1, noteFS,  // 01-06
                                 oct1, noteG,  oct1, noteGS, oct1, noteA,  oct1, noteAS, oct1, noteB, oct1, noteC,   // 07-12
                                 oct1, noteCS, oct1, noteD,  oct1, noteDS, oct1, noteE,  oct1, noteF, oct1, noteFS,  // 13-18
                                 oct1, noteG,  oct1, noteGS, oct1, noteA,  oct1, noteAS, oct1, noteB, oct1, noteC,   // 19-24
                                 oct1, noteCS, oct1, noteD,  oct1, noteDS, oct1, noteE,  oct1, noteF, oct1, noteFS,  // 25-30
                                 oct1, noteG,  oct1, noteGS, oct1, noteA,  oct1, noteAS, oct1, noteB, oct1, noteC,   // 31-36
                                 oct2, noteCS, oct2, noteD,  oct2, noteDS, oct2, noteE,  oct2, noteF, oct2, noteFS,  // 37-42
                                 oct2, noteG,  oct2, noteGS, oct2, noteA,  oct2, noteAS, oct2, noteB, // 43-47
                                 // Notes between 48 and 84 are the 37-note range of the CS15's actual keyboard
                                 oct2, noteC,   // 48 is key CL
                                 oct3, noteCS, oct3, noteD,  oct3, noteDS, oct3, noteE,  oct3, noteF, oct3, noteFS,  // 49-54  CS15
                                 oct3, noteG,  oct3, noteGS, oct3, noteA,  oct3, noteAS, oct3, noteB, oct3, noteC,   // 55-60  Physical
                                 oct4, noteCS, oct4, noteD,  oct4, noteDS, oct4, noteE,  oct4, noteF, oct4, noteFS,  // 61-66  Keyboard
                                 oct4, noteG,  oct4, noteGS, oct4, noteA,  oct4, noteAS, oct4, noteB, oct4, noteC,   // 67-72  Range
                                 oct5, noteCS, oct5, noteD,  oct5, noteDS, oct5, noteE,  oct5, noteF, oct5, noteFS,  // 73-78  From 48
                                 oct5, noteG,  oct5, noteGS, oct5, noteA,  oct5, noteAS, oct5, noteB, oct5, noteC,   // 79-84  to 84
                                 // Notes above code 84 wrap back to highest octave
                                 oct5, noteCS, oct5, noteD,  oct5, noteDS, oct5, noteE,  oct5, noteF, oct5, noteFS,  // 85-90
                                 oct5, noteG,  oct5, noteGS, oct5, noteA,  oct5, noteAS, oct5, noteB, oct5, noteC,   // 91-96
                                 oct5, noteCS, oct5, noteD,  oct5, noteDS, oct5, noteE,  oct5, noteF, oct5, noteFS,  // 97-102
                                 oct5, noteG,  oct5, noteGS, oct5, noteA,  oct5, noteAS, oct5, noteB, oct5, noteC,   // 103-108
                                 oct5, noteCS, oct5, noteD,  oct5, noteDS, oct5, noteE,  oct5, noteF, oct5, noteFS,  // 109-114
                                 oct5, noteG,  oct5, noteGS, oct5, noteA,  oct5, noteAS, oct5, noteB, oct5, noteC,   // 115-120
                                 oct5, noteCS, oct5, noteD,  oct5, noteDS, oct5, noteE,  oct5, noteF, oct5, noteFS,  // 121-126
                                 oct5, noteG };   // 127

// Static variables

volatile  uint8_t  tickCount = 0; // Timer1 set for 1ms interrupts, tally them

// Some of these can probably be shared but for now maintain separate key and midi variables
byte channel = 1; // MIDI channel, default is 1.
byte midiNoteNum;
byte midiNoteVel;
byte keyNoteNum;
byte keyNoteVel = 127; // CS15's keyboard does not do velocity sensing, just use max.value of 127
boolean keyPressed[37];
int midiNoteMapIndex;
int keyNoteMapIndex;
 // Buffer management variabes are preloaded as "clear" (no notes to play)
int midiNoteBufferIndex = 0;
int keyNoteBufferIndex = 0;
byte midiNoteBuffer[] = { 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 }; // Padded 8-note buffer for MIDI
byte keyNoteBuffer[] = { 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 };  // Padded 8-note buffer for keys
byte midiNoteToRemove = 255;
byte keyNoteToRemove = 255;

// Use a list of bit fields instead of shifting bit left every time.
// MSB is used to toggle the status LED, LSB is set high to minimize brief I/O pin
// fighting if the CL key happens to be pressed while clocking data at the 74HC595.
// (A 1K resistor (R17) is also used to buffer this possible condition.)
// Board rev 1.1 will use LSB to read six DIP switches for MIDI channel# and modes.
int bits[] = { B00000011, B00000101, B00001001, B10010001, B10100001, B11000001 }; // v1.0x
// int bits[] = { B00000010, B00000100, B00001000, B10010000, B10100000, B11000000 }; v1.1x

// scanKeybed places an active strobe on keybed group "keygroup"
// A 74HC595 is used as a cheap 8-bit output port.
// data595 must be pinMode OUTPUT before calling this
void scanKeybed(int keygroup) {
	digitalWrite(latch595, LOW); // prepare register clock
	shiftOut(data595, clock595, MSBFIRST, keygroup); //Load the port
	digitalWrite(latch595, HIGH); //Update the port data pins
}

// Flush the midiNoteBuffer, used by MIDI controller code 123 (all notes off) as
// well as initialization.
void clearMidiNoteBuffer() {
        for (int i = 0; i < 10; i++)
            midiNoteBuffer[i] = 128;
        midiNoteBufferIndex = 0;
        midiNoteToRemove = 255;
}

// Called at the end of processing MIDI note off messages to repack the midiNoteBuffer
void compactMidiNoteBuffer() {
        for (int i = 1; i < 9; i++) {
            if (midiNoteBuffer[i] == midiNoteToRemove) { // Maintain midiNoteBuffer as needed
                midiNoteBuffer[i] = 128; // Clear note entry
                midiNoteToRemove = 255; // Unflag the now-removed note
                midiNoteBufferIndex--; // Shrink filled buffer count
            }
            if (midiNoteBuffer[i] == 128) { // If current buffer location empty
                midiNoteBuffer[i] = midiNoteBuffer[i + 1]; // compact buffer
                midiNoteBuffer[i + 1] = 128; // justify with "slot empty" value
            }
        }
}

// Key down processing
void  handleKeyDown(int column, int group) {
  	keyPressed[column + group] = true;
        keyNoteNum = bGrab(midiKeyVal + column + group); // Obtain the MIDI note value for this key
        keyNoteBufferIndex++;
        keyNoteBuffer[keyNoteBufferIndex] = keyNoteNum; // Save it in the buffer
        MIDI.sendNoteOn(keyNoteNum, keyNoteVel, channel); // Send it on MIDI
        keyNoteMapIndex = keyNoteNum << 1; // Offset for DAC octave and note
        dac.output2(wGrab(octDAC + bGrab(noteMap + keyNoteMapIndex)), wGrab(noteDAC + bGrab(noteMap + keyNoteMapIndex + 1)));
        digitalWrite(gate, HIGH); // Gate on
        if (keyNoteBufferIndex == 9) { // 9 notes in buffer?  Limit it to 8
            keyNoteBuffer[1] = 128; // Flag oldest entry for removal
            keyNoteBufferIndex--; // Back out to 8th entry
        }

}

// Key up processing
void  handleKeyUp(int column, int group) {
        keyPressed[column + group] = false;
        keyNoteToRemove = bGrab(midiKeyVal + column + group); // Tag the note to remove
        MIDI.sendNoteOn(keyNoteToRemove, 0, channel); // Send the MIDI note off (note on w/velocity zero)
        if (keyNoteBuffer[keyNoteBufferIndex] == keyNoteToRemove) { // If current note is to be turned off
            if (keyNoteBufferIndex > 0) { // then provided the buffer is not empty
                if ((keyNoteBufferIndex - 1) > 0) { // and it is not the only note playing
                    keyNoteMapIndex = keyNoteBuffer[keyNoteBufferIndex - 1] << 1; // play the previous note legato style (no re-trigger)
                    dac.output2(wGrab(octDAC + bGrab(noteMap + keyNoteMapIndex)), wGrab(noteDAC + bGrab(noteMap + keyNoteMapIndex + 1)));
                }
            }
        }
        for (int i = 1; i < 9; i++) { // compact KeyNoteBuffer
            if (keyNoteBuffer[i] == keyNoteToRemove) { // Maintain keyNoteBuffer as needed
                keyNoteBuffer[i] = 128; // Clear note entry
                keyNoteToRemove = 255; // Unflag the now-removed note
                keyNoteBufferIndex--; // Shrink filled buffer count
            }
            if (keyNoteBuffer[i] == 128) { // If current buffer location empty
                keyNoteBuffer[i] = keyNoteBuffer[i + 1]; // compact buffer
                keyNoteBuffer[i + 1] = 128; // justify with "slot empty" value
            }
        }
}

// v1.1 call to read the six DIP switches.  On v1.0 boards returns all zeroes.
byte readDIPSwitches() {
      byte switches = 0;
      scanKeybed(1); // LSB is DIP switch column
      switches = digitalRead(rowFsC) << 5 | digitalRead(rowFB) << 4 | digitalRead(rowEAs) << 3 |
                      digitalRead(rowDsA) << 2 | digitalRead(rowDGs) << 1 | digitalRead(rowCsG); 
      return switches;
}

// debugLED() normally commented out, used in debugging
/* void debugLED() {
        scanKeybed(128); // LED on
        delay(100);
        scanKeybed(0); // LED off
        delay(200);
} */

void setup() {  	

	// setup pin output/input modes
	pinMode(data595, OUTPUT); // Also used as rowCL input at times
	pinMode(clock595, OUTPUT);
	pinMode(latch595, OUTPUT);

	pinMode(rowCsG, INPUT);
	pinMode(rowDGs, INPUT);
	pinMode(rowDsA, INPUT);
	pinMode(rowEAs, INPUT);
        pinMode(rowFB, INPUT);
        pinMode(rowFsC, INPUT);
//      pinMode(rowCL, INPUT); rowCL is dealt with in the main loop

        pinMode(gate, OUTPUT);

// Initialize note buffer
        clearMidiNoteBuffer();

// Blinky stuff to show we're starting up
        for (int i=0; i < 8; i++) {
        scanKeybed(128); // MSB is status LED
        delay(100);
        scanKeybed(0); // LED off
        delay(100);
        }        
// Timer1 used for 1ms interrupts
        noInterrupts(); // Global interrupt disable
        TCCR1A = 0;
        TCCR1B = 0;
        TCNT1 = 0;
        OCR1A = 624; // count = 1 /(16MHz / (100Hz * 256)) - 1
        
        TCCR1B |= (1 << WGM12); // CTC mode
        TCCR1B |= (1 << CS11) | (1 << CS10); // Timer 1 prescale /256
        TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
        interrupts(); // Global interrupt enable

// Get initial MIDI channel. v1.0 boards read back zero for default channel 1
        channel = (readDIPSwitches() & B00001111) + 1;
        
        MIDI.begin(channel); // MIDI receive on default channel 1
        MIDI.turnThruOff();

	delay(100); // Takes about this long for the CS15 to stabilize
}

// Timer1 interrupt just kicks a counter
ISR(TIMER1_COMPA_vect) {
        tickCount++;
}

void loop() {
        // Process MIDI notes, if any (each MIDI.read is one 2 or 3-byte message)
        if (MIDI.read()) {
            switch(MIDI.getType())
            {
                case midi::NoteOn:
                    midiNoteNum = MIDI.getData1();
                    midiNoteVel = MIDI.getData2();
                    if (midiNoteVel != 0) {
                        midiNoteBufferIndex++;
                        midiNoteBuffer[midiNoteBufferIndex] = midiNoteNum;  // Add newest note to buffer
                        midiNoteMapIndex = midiNoteNum << 1; // x2 for proper offset into noteMap for DAC
                        dac.output2(wGrab(octDAC + bGrab(noteMap + midiNoteMapIndex)), wGrab(noteDAC + bGrab(noteMap + midiNoteMapIndex + 1)));
                        digitalWrite(gate, HIGH); // Key on
                        if (midiNoteBufferIndex == 9) { // 9 notes in buffer?  Limit it to 8
                            midiNoteBuffer[1] = 128; // Flag oldest entry for removal
                            midiNoteBufferIndex--; // Back out to 8th entry
                        }
                    } else { // "Velocity zero" note off
                        midiNoteToRemove = midiNoteNum; // Flag note to clear from buffer
                        if (midiNoteBuffer[midiNoteBufferIndex] == midiNoteToRemove) { // If current note is to be turned off
                            if (midiNoteBufferIndex > 0) { // then provided the buffer is not empty
                                if ((midiNoteBufferIndex - 1) > 0) { // and it is not the only note playing
                                    midiNoteMapIndex = midiNoteBuffer[midiNoteBufferIndex - 1] << 1; // play the previous note legato style (no re-trigger)
                                    dac.output2(wGrab(octDAC + bGrab(noteMap + midiNoteMapIndex)), wGrab(noteDAC + bGrab(noteMap + midiNoteMapIndex + 1)));
                                }
                            }
                        }
                        compactMidiNoteBuffer(); // Manage MIDI note buffer
                    }
                    break;
                case midi::NoteOff: // Same action as velocity zero note off
                    midiNoteToRemove = MIDI.getData1(); // Flag note to clear from buffer
                    if (midiNoteBuffer[midiNoteBufferIndex] == midiNoteToRemove) { // If current note is to be turned off
                        if (midiNoteBufferIndex > 0) { // then provided the buffer is not empty
                            if ((midiNoteBufferIndex - 1) > 0) { // and it is not the only note playing
                                midiNoteMapIndex = midiNoteBuffer[midiNoteBufferIndex - 1] << 1; // play the previous note legato style (no re-trigger)
                                dac.output2(wGrab(octDAC + bGrab(noteMap + midiNoteMapIndex)), wGrab(noteDAC + bGrab(noteMap + midiNoteMapIndex + 1)));
                            }
                        }
                    }
                    compactMidiNoteBuffer(); // Manage MIDI note buffer
                    break;
                case midi::ControlChange:
                    if ( MIDI.getData1() == 123 && MIDI.getData2() == 0) // CC#123 with data2=0 is "all notes off"
                      clearMidiNoteBuffer;
                    break;
                default:
                    break;
            }
        } // End of MIDI input processing

        // Main loop of keybed scanner uses a "last note priority" scheme on the key scanner.
        // GATE is turned on with any key down, and off only when all keys are up.
        // "tickCount" is used to scan at 200Hz for a 5ms key switch debounce.
        if (tickCount > 4) {
        // CL key is checked by itself
                scanKeybed(bits[0]); // K11 column has CL in it
                pinMode(rowCL, INPUT); // Borrow data595 for use as rowCL pin for use as an input
                int groupCL = digitalRead(rowCL); // to check the CL key
                pinMode(data595, OUTPUT); // then return data595 to an output (rowCL = data595 = 15)

                // CL pressed
                if (groupCL != 0 && !keyPressed[0])
                        handleKeyDown(0, 0);
                // CL released
	        if (groupCL == 0 && keyPressed[0])
                        handleKeyUp(0, 0);
        
                // All other keys (6 groups)
	        for (int col = 0; col < 6; col++) {
		
		        // Scan a group
		        scanKeybed(bits[col]);

		        // Check if any keys were pressed - rows will have HIGH state if so
		        int groupCsG = digitalRead(rowCsG);
		        int groupDGs = digitalRead(rowDGs);
		        int groupDsA = digitalRead(rowDsA);
		        int groupEAs = digitalRead(rowEAs);
                        int groupFB = digitalRead(rowFB);
                        int groupFsC = digitalRead(rowFsC);

		        // Deal with any key(s) found pressed
		        if (groupCsG != 0 || groupDGs != 0 || groupDsA != 0
				|| groupEAs != 0 || groupFB != 0 || groupFsC != 0) {
                            // a C sharp and/or G is pressed
			    if (groupCsG != 0 && !keyPressed[col + GRP1])
                                handleKeyDown(col, GRP1);
                            // a D and/or G sharp is pressed
			    if (groupDGs != 0 && !keyPressed[col + GRP2])
                                handleKeyDown(col, GRP2);
                            // a D sharp and/or A is pressed
			    if (groupDsA != 0 && !keyPressed[col + GRP3])
                                handleKeyDown(col, GRP3);
                            // an E and/or A sharp is pressed
			    if (groupEAs != 0 && !keyPressed[col + GRP4])
                                handleKeyDown(col, GRP4);
                            // an F or B is pressed
                            if (groupFB != 0 && !keyPressed[col + GRP5])
                                handleKeyDown(col, GRP5);
                            // an F sharp or C other than CL is pressed
                            if (groupFsC != 0 && !keyPressed[col + GRP6])
                                handleKeyDown(col, GRP6);
		        }
                
		        //  Deal with MIDI note offs if any keys released
		        if (groupCsG == 0 && keyPressed[col + GRP1])
                                handleKeyUp(col, GRP1);
		        if (groupDGs == 0 && keyPressed[col + GRP2])
                                handleKeyUp(col, GRP2);
		        if (groupDsA == 0 && keyPressed[col + GRP3])
                                handleKeyUp(col, GRP3);
		        if (groupEAs == 0 && keyPressed[col + GRP4])
                                handleKeyUp(col, GRP4);
		        if (groupFB == 0 && keyPressed[col + GRP5])
                                handleKeyUp(col, GRP5);
		        if (groupFsC == 0 && keyPressed[col + GRP6])
                                handleKeyUp(col, GRP6);
                tickCount = 0; // and clear for next interval
	        }
        }
        // Check keyNoteBufferIndex, midiNoteBufferIndex and if both zero (no keys down), release the gate.
        if (keyNoteBufferIndex == 0 && midiNoteBufferIndex == 0)
                digitalWrite(gate, LOW);  //Release GATE
}                      
// END

