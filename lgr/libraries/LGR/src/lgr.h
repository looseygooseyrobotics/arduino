#ifndef LGR_H
#define LGR_H

// Loosey Goosey Robotics library for the goosebrain (RP2354A).
//
// Declare the robot's parts as globals and read the controller in loop():
//
//   #include "lgr.h"
//
//   Motor left(M3);
//   Motor right(M4, REVERSED);
//   ServoMotor claw(S1, 1000, 2000);
//
//   void setup() {}
//
//   void loop() {
//     left.move(controller.leftY() + controller.rightX());
//     right.move(controller.leftY() - controller.rightX());
//     claw.move(controller.a() ? 2000 : 1000);
//   }
//
// The library sets everything up before setup() runs, and runs the
// controller, the battery check and the status LEDs on the second core. With
// no controller plugged in, every stick, trigger and button reads 0, so a
// robot driven from the controller stops when it unplugs.
//
// A button reads true for as long as it's held. For something that should
// happen once per press, such as a toggle, add Pressed to its name:
//
//   if (controller.aPressed()) {
//     clawOpen = !clawOpen;
//   }
//
// To keep something running in the background while loop() does the driving,
// write it as a function and make a Task from it. The library calls the
// function 100 times a second:
//
//   float liftTarget = 0;
//
//   void holdLift() {
//     lift.move((liftTarget - liftEncoder.angle()) * 0.5);
//   }
//   Task liftTask(holdLift);
//
// and loop() only has to set liftTarget.
//
// Build with Tools > Board set to goosebrain, from the Loosey Goosey Robotics
// boards package, which sets every other option and brings this library and
// the ones it needs (Pico PIO USB, Adafruit NeoPixel).

#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// Ports, as labelled on the board: {IN1, IN2} of each driver. Driving a motor
// forward puts + on pin 1 of its connector, the same on all six. (The OUT1 and
// OUT2 net names are swapped on M2, M4 and M6 in the schematic, but the wiring
// isn't: OUT1 goes to pin 1 on every port.)
struct MotorPort {
    uint8_t fwd, rev;
};
constexpr MotorPort M1 = {26, 27};
constexpr MotorPort M2 = {22, 23};
constexpr MotorPort M3 = {20, 21};
constexpr MotorPort M4 = {16, 17};
constexpr MotorPort M5 = {12, 13};
constexpr MotorPort M6 = {14, 15};

// S ports on J5, for servos or encoders
constexpr int S1 = 0;
constexpr int S2 = 1;
constexpr int S3 = 2;
constexpr int S4 = 3;
constexpr int S5 = 8;
constexpr int S6 = 9;

constexpr bool REVERSED = true;

class Motor {
  public:
    Motor(MotorPort port, bool reversed = false);
    void move(int speed); // -100 to 100; 0 lets the motor coast
    void stop();          // the same as move(0)
    int speed();
    void begin();         // used by the library
  private:
    MotorPort _port;
    bool _reversed;
    int _speed = 0;
};

// A hobby servo. Positions are pulse widths in microseconds, kept between
// minPulse and maxPulse. It holds start until the first move().
class ServoMotor {
  public:
    ServoMotor(int port, int minPulse = 1000, int maxPulse = 2000, int start = -1); // start -1: the middle
    void move(int position);
    int position();
    void begin();         // used by the library
  private:
    Servo _servo;
    int _port, _min, _max;
    volatile int _position;
    volatile bool _attached = false;
};

// An AS5600 in PWM output mode on an S port. Counts past one turn in either
// direction, from when the program starts or reset() was last called.
class Encoder {
  public:
    Encoder(int port, bool reversed = false);
    float angle();     // total degrees
    float turns();     // total turns
    float position();  // 0-360 degrees within one turn
    bool connected();  // false if no reading for 50 ms
    void reset();      // zero the total
    void begin();      // used by the library
    void onEdge();     // used by the library
  private:
    int _port;
    bool _reversed;
    volatile uint32_t _rise = 0;
    volatile uint32_t _period = 0;
    volatile int _raw = -1;   // 0-4095, or -1 until a good frame
    volatile long _total = 0; // in 1/4096 turns
};

// The controller on the USB-A port: an XInput (Xbox 360 style) gamepad, such
// as an 8BitDo or a generic pad with its USB wireless dongle in X-input mode.
class Controller {
  public:
    bool connected();

    // -100 to 100, right and up positive, with a small dead zone in the middle
    int leftX();
    int leftY();
    int rightX();
    int rightY();

    // 0 to 100
    int leftTrigger();
    int rightTrigger();

    bool a();      // bottom face button
    bool b();      // right
    bool x();      // left
    bool y();      // top
    bool l1();     // bumpers
    bool r1();
    bool l2();     // triggers, pressed past a quarter
    bool r2();
    bool l3();     // stick clicks
    bool r3();
    bool up();     // d-pad
    bool down();
    bool left();
    bool right();
    bool select();
    bool start();
    bool home();

    // The same buttons, but true only once for each press, however long the
    // button is held: for toggles and for stepping through positions. A press
    // is kept until it's read, so a quick tap between two passes of loop()
    // still counts.
    bool aPressed();
    bool bPressed();
    bool xPressed();
    bool yPressed();
    bool l1Pressed();
    bool r1Pressed();
    bool l2Pressed();
    bool r2Pressed();
    bool l3Pressed();
    bool r3Pressed();
    bool upPressed();
    bool downPressed();
    bool leftPressed();
    bool rightPressed();
    bool selectPressed();
    bool startPressed();
    bool homePressed();

    // Print every input on one line, for checking a pad.
    void print();
};

extern Controller controller;

// A function that runs in the background, alongside loop(). The library calls
// it again and again, every 10 ms unless you give another time, starting when
// the robot powers on:
//
//   void holdLift() {
//     lift.move((liftTarget - liftEncoder.angle()) * 0.5);
//   }
//   Task liftTask(holdLift);
//
// The function does one step and returns; it needs no loop or delay() of its
// own, though it may use delay() (to wait for a mechanism, say) and Serial.
// loop() keeps running whatever the task does, and the other way round, so
// loop() can also wait in a delay() or a while loop without stopping a task.
// Pass values between them in global variables, like liftTarget here, and
// move each motor from only one of them.
class Task {
  public:
    Task(void (*function)(), int everyMs = 10);
    void begin();         // used by the library
    void run();           // used by the library
  private:
    void (*_function)();
    int _every;
};

// The LED stick (8 WS2812B). After a white sweep at power-on and 2 or 3 white
// LEDs for a 2S or 3S pack, the stick shows the battery as a bar from the end
// nearest J12 whenever a pack is on: green, red when low (under 3.5 V per
// cell), blinking red when too dead to drive (under 3.3 V per cell). The last
// LED blinks the status, alternating with green when the battery is full:
//   blinking purple: no pack, running on USB-C power alone
//   blinking amber: something on USB-A, but no controller to read: a dongle
//     with its pad off or unpaired, or a dongle in the wrong mode
//   blinking blue: nothing on the USB-A port
//   none: controller connected
// Set statusLeds = false to drive the stick yourself. On USB-C power alone the
// stick shares U8's 150 mA with the MCU: keep it under about 80 mA there.
extern Adafruit_NeoPixel leds;
extern volatile bool statusLeds;

// The battery. It reads 0 with the power switch off.
float batteryVoltage();
int batteryCells();  // 2 or 3, once a pack has been on for 200 ms; 0 before
bool batteryLow();   // under 3.3 V per cell: too dead to drive

#endif
