/*
 * Buzzer.c
 *
 *  Created on: Feb 8, 2021
 *      Author: beta
 */

#include "Buzzer.h"
#include "functions.h"

uint32_t melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};
//Mario main them tempo
 uint32_t tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

 uint32_t underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};
 //Underwolrd tempo
  uint32_t underworld_tempo[] = {
   12, 12, 12, 12,
   12, 12, 6,
   3,
   12, 12, 12, 12,
   12, 12, 6,
   3,
   12, 12, 12, 12,
   12, 12, 6,
   3,
   12, 12, 12, 12,
   12, 12, 6,
   6, 18, 18, 18,
   6, 6,
   6, 6,
   6, 6,
   18, 18, 18, 18, 18, 18,
   10, 10, 10,
   10, 10, 10,
   3, 3, 3
 };
void buzz(long frequency, long length) {

  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    HAL_GPIO_WritePin(BuzzerPin_GPIO_Port, BuzzerPin_Pin, GPIO_PIN_RESET);//(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delay_us(delayValue); // wait for the calculated delay value
    HAL_GPIO_WritePin(BuzzerPin_GPIO_Port, BuzzerPin_Pin, GPIO_PIN_SET);//(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delay_us(delayValue); // wait again or the calculated delay value
  }
  //digitalWrite(9, LOW);

}
void sing(int s) {
	// iterate over the notes of the melody:
	static uint8_t song;
	static uint16_t size;
	static uint32_t noteDuration;
	song=s;
	if (song == 2)
	{
		//Serial.println(" 'Underworld Theme'");
		 size = sizeof(underworld_melody) / sizeof(int);
		for (int thisNote = 0; thisNote < size; thisNote++) {

			// to calculate the note duration, take one second
			// divided by the note type.
			//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
			noteDuration = 1000 / underworld_tempo[thisNote];

			buzz(underworld_melody[thisNote], noteDuration);

			// to distinguish the notes, set a minimum time between them.
			// the note's duration + 30% seems to work well:
			int pauseBetweenNotes = noteDuration * 1.30;
			HAL_Delay(pauseBetweenNotes);

			// stop the tone playing:
			buzz(0, noteDuration);

		}

	}
	else
	{

		//Serial.println(" 'Mario Theme'");
		int size = sizeof(melody) / sizeof(int);
		for (int thisNote = 0; thisNote < size; thisNote++) {

			// to calculate the note duration, take one second
			// divided by the note type.
			//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
			int noteDuration = 1000 / tempo[thisNote];

			buzz(melody[thisNote], noteDuration);

			// to distinguish the notes, set a minimum time between them.
			// the note's duration + 30% seems to work well:
			int pauseBetweenNotes = noteDuration * 1.30;
			HAL_Delay(pauseBetweenNotes);

			// stop the tone playing:
			buzz(0, noteDuration);

		}
	}
}
