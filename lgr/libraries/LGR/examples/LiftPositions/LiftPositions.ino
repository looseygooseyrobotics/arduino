// A lift on M5 with an encoder on IO2. Up and down on the d-pad step it
// through a list of heights, and a task in the background drives the motor
// to hold whichever height is chosen. A claw servo on IO1 opens and closes
// with each press of A. The drive is on M1 and M2, as in ArcadeDrive.

#include <lgr.h>

Motor left(M1);
Motor right(M2, REVERSED);
Motor lift(M5);
Encoder liftEncoder(IO2);
ServoMotor claw(IO1, 1000, 2000, 1000);

// lift heights, in degrees of the encoder from where the lift was at power-on
const float liftHeights[] = {0, 400, 900};
const int liftTop = 2; // the last one
int liftHeight = 0;    // which one the lift is at, or on its way to
float liftTarget = 0;

bool clawOpen = false;

// Drives the lift towards liftTarget, harder the further away it is. Raise
// liftPower if the lift is slow or sags, lower it if it shakes.
const float liftPower = 0.5;

void holdLift() {
    float error = liftTarget - liftEncoder.angle();
    lift.move(error * liftPower);
}
Task liftTask(holdLift); // runs holdLift() in the background, 100 times a second

void setup() {}

void loop() {
    // lift: one height up or down for each press
    if (controller.upPressed() && liftHeight < liftTop) {
        liftHeight++;
    }
    if (controller.downPressed() && liftHeight > 0) {
        liftHeight--;
    }
    liftTarget = liftHeights[liftHeight];

    // claw: A opens it, A again closes it
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
