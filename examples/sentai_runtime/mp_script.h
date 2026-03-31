// Embedded MicroPython blink script for SentAI board
// Blinks the user LED 10 times (500ms on / 500ms off)

#ifndef MP_SCRIPT_H_
#define MP_SCRIPT_H_

static const char mp_blink_script[] =
    "import sentai\n"
    "print('[Python] Blink script started')\n"
    "for i in range(10):\n"
    "    sentai.io.led_on()\n"
    "    sentai.rtos.sleep_ms(500)\n"
    "    sentai.io.led_off()\n"
    "    sentai.rtos.sleep_ms(500)\n"
    "    print('[Python] Blink', i + 1)\n"
    "print('[Python] Blink script done')\n";

#endif  // MP_SCRIPT_H_
