#include <lgr.h>

void setup() {}

void loop() {
    if (controller.connected()) {
        controller.print();
    } else {
        Serial.println("No controller");
    }
    delay(100);
}
