# Appendix: sentai.io

The `sentai.io` namespace groups the simplest board-local control primitives. In the current runtime it is intentionally small and is used mainly for visual status indication, quick feedback during debugging, and minimal interaction from scripts that do not require a richer GPIO abstraction.

## Functions

- `sentai.io.led_on()`: Turns the user LED on.
- `sentai.io.led_off()`: Turns the user LED off.

## Example

```python
import sentai

for _ in range(3):
    sentai.io.led_on()
    sentai.rtos.sleep_ms(200)
    sentai.io.led_off()
    sentai.rtos.sleep_ms(200)
```