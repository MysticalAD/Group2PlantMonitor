// music.h

#ifndef MUSIC_H
#define MUSIC_H

#include <Arduino.h>

// Function to play a success tune on the buzzer
void playSuccessTune(int buzzerChannel)
{
    const int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
    const int noteDurations[] = {4, 4, 4, 4, 4, 4, 4, 4}; // All notes are quarter notes

    for (int i = 0; i < 8; i++)
    {
        int noteDuration = 1000 / noteDurations[i];
        ledcWriteTone(buzzerChannel, melody[i]);
        delay(noteDuration);
        ledcWriteTone(buzzerChannel, 0); // Turn off the buzzer between notes
        delay(noteDuration * 0.3);
    }
}

#endif // MUSIC_H
