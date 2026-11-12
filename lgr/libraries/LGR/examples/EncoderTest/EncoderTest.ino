// Prints what the encoder on S1 reads, ten times a second. Open
// Tools > Serial Monitor and turn the shaft by hand.
//
//   angle:    total degrees since power-on, past one turn in either direction
//   turns:    the same, in turns
//   position: 0 to 360 degrees within one turn, like a compass

#include <lgr.h>

Encoder encoder(S1);

void setup() {}

void loop() {
    if (controller.aPressed()) {
        encoder.reset();
    }

    if (encoder.connected()) {
        Serial.printf("angle %8.1f  turns %6.2f  position %5.1f\n",
                      encoder.angle(), encoder.turns(), encoder.position());
    } else {
        Serial.println("No encoder on S1");
    }
    delay(100);
}
