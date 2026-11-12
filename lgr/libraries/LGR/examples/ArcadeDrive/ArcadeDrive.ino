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
