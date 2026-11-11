// Two drive motors on M1 and M2, driven from the controller: the left stick
// goes forward and back, the right stick turns. A claw servo on S1 closes
// while A is held.

#include <lgr.h>

Motor left(M1);
Motor right(M2, REVERSED);
ServoMotor claw(S1, 1000, 2000, 1000);

void setup() {}

void loop() {
    int forward = controller.leftY();
    int turn = controller.rightX();
    left.move(forward + turn);
    right.move(forward - turn);

    claw.move(controller.a() ? 2000 : 1000);

    delay(10);
}
