// goosebrain board startup, before setup() runs.

#include <Arduino.h>
#include "RP2040Version.h"
#include "hardware/structs/powman.h"

// The oldest Pico core the package has been built with. Newer ones should work.
#if ARDUINO_PICO_MAJOR < 5 || (ARDUINO_PICO_MAJOR == 5 && ARDUINO_PICO_MINOR < 7)
#error "goosebrain needs version 5.7.0 or newer of the Raspberry Pi Pico/RP2040/RP2350 boards (Tools > Board > Boards Manager)"
#endif

// The lgr library's startup, if the sketch uses it
void lgrBegin() __attribute__((weak));

// The core runs from an external 1.1 V LDO, so shut the on-chip regulator
// down (RP2350 datasheet 6.3.7). It starts anyway and draws about 400 uA
// until this runs. Never use its low-power mode, which connects it to DVDD.
static void vregHiZ() {
    hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);
    while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS) {
        tight_loop_contents();
    }
    hw_set_bits(&powman_hw->vreg, POWMAN_PASSWORD_BITS | POWMAN_VREG_HIZ_BITS);
    while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS) {
        tight_loop_contents();
    }
}

// Called by the core on core0 before setup(), and before core1 starts
void initVariant() {
    vregHiZ();
    if (lgrBegin) {
        lgrBegin();
    }
}
