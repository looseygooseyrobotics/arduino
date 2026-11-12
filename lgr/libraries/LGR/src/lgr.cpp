#include <Arduino.h>
#include "pio_usb.h"
#include "lgr.h"

#include "Adafruit_TinyUSB.h"
#include "host/usbh_pvt.h"
#include "pico/critical_section.h"

#ifndef ARDUINO_GOOSEBRAIN
#error "Set Tools > Board to goosebrain (from the Loosey Goosey Robotics boards)"
#endif

#include <FreeRTOS.h>
#include <task.h>

#define LED_PIN 29
#define LED_COUNT 8
#define LED_BRIGHTNESS 40
#define VSYS_PIN 28    // battery sense, VSYS / 4.7
#define USB_HOST_DP 25 // USB-A D+; D- is GPIO24

#define PACK_MIN 5.0
#define CELL_LOW 3.3   // dead battery cutoff
#define CELL_WARN 3.5  // the battery bar turns red below this
#define CELL_FULL 4.2
#define ENCODER_TIMEOUT 50000 // us without a rising edge: no encoder
#define STICK_DEAD_ZONE 8     // percent
#define TRIGGER_PRESS 64      // l2()/r2() press past this (of 255)
#define MAX_PARTS 6
#define MAX_TASKS 8
#define TASK_STACK 2048

Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile bool statusLeds = true;
Controller controller;

static Motor *motors[MAX_PARTS];
static ServoMotor *servos[MAX_PARTS];
static Encoder *encoders[MAX_PARTS];
static Task *tasks[MAX_TASKS];
static int motorCount, servoCount, encoderCount, taskCount;

static bool started = false;
static volatile bool servosReady = false;
static critical_section_t lock;

// Motor

Motor::Motor(MotorPort port, bool reversed) : _port(port), _reversed(reversed) {
  if (started) {
    begin();
  } else {
    motors[motorCount++] = this;
  }
}

void Motor::begin() {
  pinMode(_port.fwd, OUTPUT);
  pinMode(_port.rev, OUTPUT);
  digitalWrite(_port.fwd, LOW);
  digitalWrite(_port.rev, LOW);
}

void Motor::move(int speed) {
  speed = constrain(speed, -100, 100);
  _speed = speed;
  if (_reversed) {
    speed = -speed;
  }

  // stopped: both inputs low, so the motor coasts and the driver sleeps
  if (speed == 0) {
    digitalWrite(_port.fwd, LOW);
    digitalWrite(_port.rev, LOW);
    return;
  }

  // Drive/brake PWM: hold one input high and PWM the other, so the off time
  // brakes. A DRV8251A sleeps after about 1 ms with both inputs low and takes
  // 250 us to wake, so the off time must never coast.
  int off = 255 - map(abs(speed), 0, 100, 0, 255);
  if (speed > 0) {
    digitalWrite(_port.fwd, HIGH);
    analogWrite(_port.rev, off);
  } else {
    digitalWrite(_port.rev, HIGH);
    analogWrite(_port.fwd, off);
  }
}

void Motor::stop() {
  move(0);
}

int Motor::speed() {
  return _speed;
}

// ServoMotor

ServoMotor::ServoMotor(int port, int minPulse, int maxPulse, int start) : _port(port), _min(minPulse), _max(maxPulse) {
  _position = start < 0 ? (minPulse + maxPulse) / 2 : constrain(start, minPulse, maxPulse);
  if (!started) {
    if (servoCount < MAX_PARTS) {
      servos[servoCount++] = this;
    }
    return;
  }
  // made after startup: attach now if core1 has started, or leave it to core1
  critical_section_enter_blocking(&lock);
  if (servosReady) {
    begin();
  } else if (servoCount < MAX_PARTS) {
    servos[servoCount++] = this;
  }
  critical_section_exit(&lock);
}

void ServoMotor::begin() {
  // the pulses come from PIO, so any S port works
  _servo.attach(_port, _min, _max, _position);
  _attached = true;
}

void ServoMotor::move(int position) {
  _position = constrain(position, _min, _max);
  if (_attached) {
    _servo.writeMicroseconds(_position);
  }
}

int ServoMotor::position() {
  return _position;
}

// Encoder

static void encoderEdge(void *param) {
  ((Encoder *) param)->onEdge();
}

Encoder::Encoder(int port, bool reversed) : _port(port), _reversed(reversed) {
  if (started) {
    begin();
  } else if (encoderCount < MAX_PARTS) {
    encoders[encoderCount++] = this;
  }
}

void Encoder::begin() {
  _rise = time_us_32();
  pinMode(_port, INPUT_PULLDOWN);
  attachInterruptParam(_port, encoderEdge, CHANGE, this);
}

void Encoder::onEdge() {
  uint32_t now = time_us_32();

  if (gpio_get(_port)) {
    // a period that runs from a stale edge is no good: wait for the next one
    uint32_t period = now - _rise;
    _period = period < ENCODER_TIMEOUT ? period : 0;
    _rise = now;
    return;
  }

  uint32_t period = _period;
  uint32_t high = now - _rise;
  if (period == 0 || high >= period) {
    return;
  }
  int raw = (int) (((uint64_t) high * 4351 + period / 2) / period) - 128;
  raw = constrain(raw, 0, 4095);

  if (_raw >= 0) {
    int delta = raw - _raw;
    if (delta > 2048) {
      delta -= 4096;
    } else if (delta < -2048) {
      delta += 4096;
    }
    if (_reversed) {
      delta = -delta;
    }
    _total += delta;

    // Smoothing: an exponential average with a time constant of _smoothUs,
    // stepped by each frame's period so it's the same at any AS5600
    // frequency. It's kept as how far it lags _total, which stays small, so
    // it loses no precision however many turns the total runs to.
    float smooth = _smoothUs;
    _lag = (_lag - delta) * (smooth / (smooth + period));
  }
  _raw = raw;
}

bool Encoder::connected() {
  return _raw >= 0 && time_us_32() - _rise < ENCODER_TIMEOUT;
}

float Encoder::total() {
  // both from the same frame
  uint32_t irq = save_and_disable_interrupts();
  long total = _total;
  float lag = _lag;
  restore_interrupts(irq);
  return total + lag;
}

float Encoder::angle() {
  return total() * 360.0 / 4096;
}

float Encoder::turns() {
  return total() / 4096.0;
}

void Encoder::smoothing(int ms) {
  _smoothUs = max(ms, 0) * 1000.0f;
}

float Encoder::position() {
  int raw = _raw;
  if (raw < 0) {
    return 0;
  }
  if (_reversed) {
    raw = (4096 - raw) % 4096;
  }
  return raw * 360.0 / 4096;
}

void Encoder::reset() {
  uint32_t irq = save_and_disable_interrupts();
  _total = 0;
  _lag = 0;
  restore_interrupts(irq);
}

// Task

static void taskThread(void *param) {
  ((Task *) param)->run();
}

Task::Task(void (*function)(), int everyMs) : _function(function), _every(max(everyMs, 1)) {
  if (started) {
    begin();
  } else if (taskCount < MAX_TASKS) {
    tasks[taskCount++] = this;
  }
}

void Task::begin() {
  // loop()'s priority (see the core's freertos-main.cpp), and loop()'s core
  if (_function == nullptr ||
      xTaskCreateAffinitySet(taskThread, "Task", TASK_STACK, this, configMAX_PRIORITIES / 2, 1 << 0, nullptr) != pdPASS) {
    Serial.println("Task: can't start");
  }
}

void Task::run() {
  TickType_t last = xTaskGetTickCount();
  while (true) {
    _function();
    // a function that took longer than its time starts again from now, after
    // a tick for loop(), rather than running flat out to catch up
    if (!xTaskDelayUntil(&last, pdMS_TO_TICKS(_every))) {
      vTaskDelay(1);
      last = xTaskGetTickCount();
    }
  }
}

// Controller

enum {
  BTN_A = 1 << 0, BTN_B = 1 << 1, BTN_X = 1 << 2, BTN_Y = 1 << 3,
  BTN_L1 = 1 << 4, BTN_R1 = 1 << 5, BTN_L2 = 1 << 6, BTN_R2 = 1 << 7,
  BTN_L3 = 1 << 8, BTN_R3 = 1 << 9,
  BTN_UP = 1 << 10, BTN_DOWN = 1 << 11, BTN_LEFT = 1 << 12, BTN_RIGHT = 1 << 13,
  BTN_SELECT = 1 << 14, BTN_START = 1 << 15, BTN_HOME = 1 << 16,
};

struct PadState {
  uint32_t buttons;       // BTN_*
  int16_t lx, ly, rx, ry; // -32768 to 32767, right and up positive
  uint8_t lt, rt;         // 0-255
};

static PadState pad;
static bool padConnected = false;
static bool padReported = false;
static uint32_t padPressed = 0;

static Adafruit_USBH_Host USBHost;

static void setPad(bool connected, const PadState &s) {
  critical_section_enter_blocking(&lock);
  padConnected = connected;
  padPressed = connected ? padPressed | (s.buttons & ~pad.buttons) : 0;
  pad = s;
  critical_section_exit(&lock);
}

static void padMounted() {
  Serial.println("Controller is connected");
  padReported = false;
  setPad(true, PadState{});
}

static void padUnmounted() {
  Serial.println("Controller disconnected");
  padReported = false;
  setPad(false, PadState{});
}

// a trigger past a quarter also presses l2()/r2()
static void setPadReport(PadState &s) {
  s.buttons |= (s.lt > TRIGGER_PRESS ? BTN_L2 : 0) | (s.rt > TRIGGER_PRESS ? BTN_R2 : 0);
  padReported = true;
  setPad(true, s);
}

//--------------------------------------------------------------------+
// XInput class driver
//--------------------------------------------------------------------+

static const uint32_t XINPUT_BUTTONS[16] = {
  BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_START, BTN_SELECT, BTN_L3, BTN_R3,
  BTN_L1, BTN_R1, BTN_HOME, 0, BTN_A, BTN_B, BTN_X, BTN_Y,
};

static struct {
  bool mounted;
  uint8_t daddr, itf, epIn, epOut;
  uint8_t inBuf[32];
  uint8_t outBuf[3];
} xin;

static bool xinputReceive() {
  if (!usbh_edpt_claim(xin.daddr, xin.epIn)) {
    return false;
  }
  if (!usbh_edpt_xfer(xin.daddr, xin.epIn, xin.inBuf, sizeof(xin.inBuf))) {
    usbh_edpt_release(xin.daddr, xin.epIn);
    return false;
  }
  return true;
}

static bool xinputInit() {
  memset(&xin, 0, sizeof(xin));
  return true;
}

static bool xinputDeinit() {
  return true;
}

static uint16_t xinputOpen(uint8_t rhport, uint8_t daddr, tusb_desc_interface_t const *itf, uint16_t maxLen) {
  (void) rhport;
  if (xin.mounted || itf->bInterfaceClass != 0xFF || itf->bInterfaceSubClass != 0x5D ||
      itf->bInterfaceProtocol != 0x01) {
    return 0;
  }

  uint8_t const *p = (uint8_t const *) itf;
  uint8_t const *end = p + maxLen;
  uint16_t len = tu_desc_len(p);
  p = tu_desc_next(p);
  while (p < end && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
    if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
      tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p;
      TU_ASSERT(tuh_edpt_open(daddr, ep), 0);
      if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
        xin.epIn = ep->bEndpointAddress;
      } else {
        xin.epOut = ep->bEndpointAddress;
      }
    }
    len += tu_desc_len(p);
    p = tu_desc_next(p);
  }

  xin.daddr = daddr;
  xin.itf = itf->bInterfaceNumber;
  return xin.epIn ? len : 0;
}

static bool xinputSetConfig(uint8_t daddr, uint8_t itf) {
  xin.mounted = true;
  padMounted();

  // light the player 1 LED (some pads wait for this before they report)
  if (xin.epOut && usbh_edpt_claim(daddr, xin.epOut)) {
    xin.outBuf[0] = 0x01;
    xin.outBuf[1] = 0x03;
    xin.outBuf[2] = 0x06;
    if (!usbh_edpt_xfer(daddr, xin.epOut, xin.outBuf, 3)) {
      usbh_edpt_release(daddr, xin.epOut);
    }
  }

  if (!xinputReceive()) {
    Serial.println("Controller: can't start reading reports");
  }
  usbh_driver_set_config_complete(daddr, itf);
  return true;
}

static bool xinputXferCb(uint8_t daddr, uint8_t ep, xfer_result_t result, uint32_t len) {
  (void) daddr;
  if (ep != xin.epIn) {
    return true;
  }

  // input report: type 0x00, length 0x14, buttons, LT, RT, LX, LY, RX, RY
  uint8_t const *r = xin.inBuf;
  if (result == XFER_RESULT_SUCCESS && len >= 14 && r[0] == 0x00 && r[1] == 0x14) {
    PadState s = {};
    uint16_t bits = r[2] | (r[3] << 8);
    for (int n = 0; n < 16; n++) {
      if (bits & (1 << n)) {
        s.buttons |= XINPUT_BUTTONS[n];
      }
    }
    s.lt = r[4];
    s.rt = r[5];
    s.lx = (int16_t) (r[6] | (r[7] << 8));
    s.ly = (int16_t) (r[8] | (r[9] << 8));
    s.rx = (int16_t) (r[10] | (r[11] << 8));
    s.ry = (int16_t) (r[12] | (r[13] << 8));
    setPadReport(s);
  }

  xinputReceive();
  return true;
}

static void xinputClose(uint8_t daddr) {
  if (xin.mounted && xin.daddr == daddr) {
    memset(&xin, 0, sizeof(xin));
    padUnmounted();
  }
}

static usbh_class_driver_t const xinputDriver = {
  "XInput",
  xinputInit,
  xinputDeinit,
  xinputOpen,
  xinputSetConfig,
  xinputXferCb,
  xinputClose,
};

extern "C" const usbh_class_driver_t *usbh_app_driver_get_cb(uint8_t *count) {
  *count = 1;
  return &xinputDriver;
}

static uint8_t strangerAddr = 0;

extern "C" void tuh_mount_cb(uint8_t daddr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(daddr, &vid, &pid);
  Serial.printf("USB device %04x:%04x plugged in\n", vid, pid);
  if (!xin.mounted) {
    Serial.println("  it isn't an XInput controller, so it can't be read");
    strangerAddr = daddr;
  }
}

extern "C" void tuh_umount_cb(uint8_t daddr) {
  if (daddr == strangerAddr) {
    strangerAddr = 0;
  }
}

//--------------------------------------------------------------------+
// Controller methods (core0)
//--------------------------------------------------------------------+

static PadState padNow() {
  critical_section_enter_blocking(&lock);
  PadState s = pad;
  critical_section_exit(&lock);
  return s;
}

static bool held(uint32_t button) {
  return padNow().buttons & button;
}

// true once for each press: reading it clears it
static bool pressed(uint32_t button) {
  critical_section_enter_blocking(&lock);
  bool p = padPressed & button;
  padPressed &= ~button;
  critical_section_exit(&lock);
  return p;
}

// -100 to 100, with the dead zone taken out so the stick still reaches 100
static int stick(int16_t v) {
  int p = constrain(v * 100 / 32767, -100, 100);
  if (abs(p) <= STICK_DEAD_ZONE) {
    return 0;
  }
  int sign = p > 0 ? 1 : -1;
  return sign * (abs(p) - STICK_DEAD_ZONE) * 100 / (100 - STICK_DEAD_ZONE);
}

bool Controller::connected() {
  critical_section_enter_blocking(&lock);
  bool c = padConnected;
  critical_section_exit(&lock);
  return c;
}

int Controller::leftX() { return stick(padNow().lx); }
int Controller::leftY() { return stick(padNow().ly); }
int Controller::rightX() { return stick(padNow().rx); }
int Controller::rightY() { return stick(padNow().ry); }
int Controller::leftTrigger() { return padNow().lt * 100 / 255; }
int Controller::rightTrigger() { return padNow().rt * 100 / 255; }

bool Controller::a() { return held(BTN_A); }
bool Controller::b() { return held(BTN_B); }
bool Controller::x() { return held(BTN_X); }
bool Controller::y() { return held(BTN_Y); }
bool Controller::l1() { return held(BTN_L1); }
bool Controller::r1() { return held(BTN_R1); }
bool Controller::l2() { return held(BTN_L2); }
bool Controller::r2() { return held(BTN_R2); }
bool Controller::l3() { return held(BTN_L3); }
bool Controller::r3() { return held(BTN_R3); }
bool Controller::up() { return held(BTN_UP); }
bool Controller::down() { return held(BTN_DOWN); }
bool Controller::left() { return held(BTN_LEFT); }
bool Controller::right() { return held(BTN_RIGHT); }
bool Controller::select() { return held(BTN_SELECT); }
bool Controller::start() { return held(BTN_START); }
bool Controller::home() { return held(BTN_HOME); }

bool Controller::aPressed() { return pressed(BTN_A); }
bool Controller::bPressed() { return pressed(BTN_B); }
bool Controller::xPressed() { return pressed(BTN_X); }
bool Controller::yPressed() { return pressed(BTN_Y); }
bool Controller::l1Pressed() { return pressed(BTN_L1); }
bool Controller::r1Pressed() { return pressed(BTN_R1); }
bool Controller::l2Pressed() { return pressed(BTN_L2); }
bool Controller::r2Pressed() { return pressed(BTN_R2); }
bool Controller::l3Pressed() { return pressed(BTN_L3); }
bool Controller::r3Pressed() { return pressed(BTN_R3); }
bool Controller::upPressed() { return pressed(BTN_UP); }
bool Controller::downPressed() { return pressed(BTN_DOWN); }
bool Controller::leftPressed() { return pressed(BTN_LEFT); }
bool Controller::rightPressed() { return pressed(BTN_RIGHT); }
bool Controller::selectPressed() { return pressed(BTN_SELECT); }
bool Controller::startPressed() { return pressed(BTN_START); }
bool Controller::homePressed() { return pressed(BTN_HOME); }

void Controller::print() {
  static const char *const names[] = {
    "A", "B", "X", "Y", "L1", "R1", "L2", "R2", "L3", "R3",
    "up", "down", "left", "right", "select", "start", "home",
  };
  PadState s = padNow();
  Serial.printf("left %4d %4d  right %4d %4d  triggers %3d %3d  buttons:",
                stick(s.lx), stick(s.ly), stick(s.rx), stick(s.ry), s.lt * 100 / 255, s.rt * 100 / 255);
  for (int n = 0; n < 17; n++) {
    if (s.buttons & (1 << n)) {
      Serial.printf(" %s", names[n]);
    }
  }
  Serial.println();
}

// Battery (core1)

static volatile float vsys = 0; // filtered battery voltage
static volatile int cells = 0;
static uint32_t packSeen = 0;   // millis() when the pack was first seen, for the startup sweep

static float readVsys() {
  return analogRead(VSYS_PIN) * 3.3 / 4095 * 4.7;
}

float batteryVoltage() {
  return vsys;
}

int batteryCells() {
  return cells;
}

bool batteryLow() {
  return cells > 0 && vsys >= PACK_MIN && vsys < CELL_LOW * cells;
}

static void updateBattery() {
  static bool warned = false;
  static bool rising = false;
  static uint32_t risingSince = 0;

  // unfiltered until the cell count is in, so it follows VSYS up at switch-on
  float v = readVsys();
  vsys = (cells == 0 || vsys < PACK_MIN || v < PACK_MIN) ? v : vsys + (v - vsys) * 0.05;

  if (cells == 0) {
    if (v < PACK_MIN) {
      rising = false;
    } else if (!rising) {
      rising = true;
      risingSince = millis();
    } else if (millis() - risingSince >= 200) {
      cells = v > 8.6 ? 3 : 2;
      packSeen = millis();
      Serial.printf("%dS pack, %.2f V\n", (int) cells, v);
    }
  }

  if (batteryLow() && !warned) {
    Serial.printf("Battery low: %.2f V\n", (float) vsys);
    warned = true;
  }
}

// Status LEDs

static void updateLeds() {
  uint32_t now = millis();
  bool blink = (now / 250) % 2;
  uint32_t green = leds.Color(0, 255, 0);
  leds.clear();

  // startup: a sweep (400 ms), then one LED per cell (1 s)
  uint32_t sinceStart = now - packSeen;
  if (cells > 0 && sinceStart < 1400) {
    if (sinceStart < 400) {
      leds.setPixelColor(sinceStart / 50, leds.Color(255, 255, 255));
    } else {
      leds.fill(leds.Color(255, 255, 255), 0, cells);
    }
    leds.show();
    return;
  }

  int n = 0;
  if (cells > 0 && vsys >= PACK_MIN) {
    float perCell = vsys / cells;
    float level = (perCell - CELL_LOW) / (CELL_FULL - CELL_LOW);
    n = constrain((int) lroundf(level * LED_COUNT), 1, LED_COUNT);
    if (batteryLow()) {
      if (blink) {
        leds.fill(leds.Color(255, 0, 0), 0, n);
      }
    } else {
      leds.fill(perCell < CELL_WARN ? leds.Color(255, 0, 0) : green, 0, n);
    }
  }

  // the status
  uint32_t status = 0;
  if (vsys < PACK_MIN) {
    status = leds.Color(160, 0, 255);
  } else if (strangerAddr || (controller.connected() && !padReported)) {
    status = leds.Color(255, 100, 0);
  } else if (!controller.connected()) {
    status = leds.Color(0, 0, 255);
  }
  if (status) {
    int last = LED_COUNT - 1;
    if (n == LED_COUNT) {
      leds.setPixelColor(last, blink ? status : green);  // full: status and green in turn
    } else if (blink) {
      leds.setPixelColor(last, status);
    }
  }

  leds.show();
}

// Startup

void lgrBegin() {
  critical_section_init(&lock);

  analogWriteFreq(20000);
  analogWriteRange(255);
  for (int i = 0; i < motorCount; i++) {
    motors[i]->begin();
  }
  for (int i = 0; i < encoderCount; i++) {
    encoders[i]->begin();
  }
  for (int i = 0; i < taskCount; i++) {
    tasks[i]->begin();
  }
  started = true;
}

// Core1: the USB host and PIO.

void setup1() {
  pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
  cfg.pin_dp = USB_HOST_DP;
  cfg.pinout = PIO_USB_PINOUT_DMDP;
  USBHost.configure_pio_usb(1, &cfg);
  USBHost.begin(1);

  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);
  leds.clear();
  leds.show();

  critical_section_enter_blocking(&lock);
  for (int i = 0; i < servoCount; i++) {
    servos[i]->begin();
  }
  servosReady = true;
  critical_section_exit(&lock);

  analogReadResolution(12);
  vsys = readVsys();
  if (vsys < PACK_MIN) {
    Serial.println("No battery detected");
  }
}

void loop1() {
  static uint32_t lastBattery = 0;
  static uint32_t lastLeds = 0;

  USBHost.task();

  if (millis() - lastBattery >= 10) {
    updateBattery();
    lastBattery = millis();
  }
  if (statusLeds && millis() - lastLeds >= 50) {
    updateLeds();
    lastLeds = millis();
  }
}
