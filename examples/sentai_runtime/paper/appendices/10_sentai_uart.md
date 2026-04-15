# Appendix: sentai.uart

The `sentai.uart` namespace exposes raw UART serial I/O on the board-side serial port when that port is not reserved for the REPL or another protocol bridge. It is intended for direct communication with external serial devices such as radios, sensors, GPS modules, or custom peripherals.

## Functions

- `sentai.uart.open(baud)`: Opens the UART port at the requested baud rate.
- `sentai.uart.close()`: Closes the UART port and restores the default configuration.
- `sentai.uart.write(data)`: Sends bytes or text.
- `sentai.uart.read(max, timeout_ms)`: Reads bytes with polling, timeout, or blocking behavior.
- `sentai.uart.available()`: Returns the number of waiting bytes.

## Example

```python
import sentai

sentai.uart.open(38400)
sentai.uart.write(b'AT\r\n')
resp = sentai.uart.read(256, 1000)
print(resp)
sentai.uart.close()
```