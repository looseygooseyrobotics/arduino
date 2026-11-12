# Goosebrain with Arduino

## Install (Arduino IDE)

1. **File > Preferences > Additional boards manager URLs**, add both (one per line):

   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   https://arduino.looseygooseyrobotics.com/package_lgr_index.json
   ```

   The first is the Raspberry Pi Pico core, which the goosebrain builds on. Skip it if you already have it.

2. **Tools > Board > Boards Manager**:
   - install the latest **Raspberry Pi Pico/RP2040/RP2350** (5.7.0 or newer)
   - install **Loosey Goosey Robotics**
3. **Tools > Board > Loosey Goosey Robotics > goosebrain**.

Don't install the **Adafruit TinyUSB** library from the Library Manager. The Pico core has its own, and a second copy
breaks the build.

## Use it in a sketch

Include the library, declare the robot's parts as globals, and read the controller in `loop()`:

```cpp
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
```

More in **File > Examples > LGR**: **ArcadeDrive** and **LiftPositions**.

## Upload

Plug the goosebrain in over USB-C and click **Upload**. The first time, or if the board stops showing a serial port:
switch the board off, hold **BOOT** while plugging in USB-C, then upload. It shows up as a drive, and the upload finds it.

## arduino-cli

```
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli config add board_manager.additional_urls https://arduino.looseygooseyrobotics.com/package_lgr_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040 lgr:rp2040
arduino-cli compile -b lgr:rp2040:goosebrain MySketch
```
