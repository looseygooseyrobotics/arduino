// Runs each motor port forward/back
#include <lgr.h>

Motor motors[] = {Motor(M1), Motor(M2), Motor(M3), Motor(M4), Motor(M5), Motor(M6)};

const int speed = 50;

void setup() {}

void loop() {
    for (int i = 0; i < 6; i++) {
        motors[i].move(speed);
        delay(1000);
    }

    for (int i = 0; i < 6; i++) {
        motors[i].move(-speed);
        delay(1000);
    }

    for (int i = 0; i < 6; i++) {
        motors[i].stop();
        delay(500);
    }
}
