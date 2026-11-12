#ifndef LGR_H
#define LGR_H

// Loosey Goosey Robotics library for the goosebrain (RP2354A).

#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

struct MotorPort {
    uint8_t fwd, rev;
};
constexpr MotorPort M1 = {26, 27};
constexpr MotorPort M2 = {22, 23};
constexpr MotorPort M3 = {20, 21};
constexpr MotorPort M4 = {16, 17};
constexpr MotorPort M5 = {12, 13};
constexpr MotorPort M6 = {14, 15};

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

// Positions are pulse widths in microseconds, kept between
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
//
// angle() and turns() are smoothed, to take out the sensor's jitter: they
// catch up with a move in about 20 ms. smoothing() changes how long.
class Encoder {
  public:
    Encoder(int port, bool reversed = false);
    float angle();     // total degrees
    float turns();     // total turns
    float position();  // 0-360 degrees within one turn, as read (not smoothed)
    bool connected();  // false if no reading for 50 ms
    void reset();      // zero the total
    void smoothing(int ms); // more is steadier but slower to follow; 0 for none
    void begin();      // used by the library
    void onEdge();     // used by the library
  private:
    float total();     // smoothed, in 1/4096 turns
    int _port;
    bool _reversed;
    volatile uint32_t _rise = 0;
    volatile uint32_t _period = 0;
    volatile int _raw = -1;   // 0-4095, or -1 until a good frame
    volatile long _total = 0; // in 1/4096 turns, as read
    volatile float _lag = 0;  // smoothed total minus _total
    volatile float _smoothUs = 20000;
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
    // button is held: for toggles and for stepping through positions.
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

    // Print every input on one line
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
// cell), blinking red when dead (under 3.3 V per cell). 
// 
// The last
// LED blinks the status, alternating with green when the battery is full:
//   blinking purple: no pack, running on USB-C power alone
//   blinking amber: something on USB-A, but no controller to read: a dongle
//     with its controller off or unpaired, or a dongle in the wrong mode
//   blinking blue: nothing on the USB-A port
// Set statusLeds = false to drive the stick yourself.
extern Adafruit_NeoPixel leds;
extern volatile bool statusLeds;

float batteryVoltage();
int batteryCells();  // 2 or 3, depending on what type of LiPo is plugged in
bool batteryLow();   // under 3.3 V per cell

#endif
