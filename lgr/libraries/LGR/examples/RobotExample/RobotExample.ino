#include <lgr.h>

Motor left(M1);
Motor right(M2, REVERSED);
Motor lift(M5);
Motor intake(M6);
Encoder liftEncoder(S2);
ServoMotor claw(S1, 1000, 2000, 1000);


bool clawOpen = false;

// Drives the lift towards liftTarget
const float kP = 0.5;
float liftTarget = 0;

void liftPid() {
    float error = liftTarget - liftEncoder.angle();
    lift.move(error * kP);
}
Task liftTask(liftPid); // runs liftPid() in the background

void setup() {}

void loop() {
    // lift
    if (controller.upPressed()) {
        liftTarget = 1000;
    } else if (controller.downPressed()) {
        liftTarget = 0;
    }

    // claw
    if (controller.aPressed()) {
        clawOpen = !clawOpen;
    }
    claw.move(clawOpen ? 2000 : 1000);

    // arcade drive
    int forward = controller.leftY();
    int turn = controller.rightX();
    left.move(forward + turn);
    right.move(forward - turn);

    delay(10);
}
