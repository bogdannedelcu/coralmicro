# Appendix: sentai.console

The `sentai.console` interface controls where the interactive MicroPython REPL is exposed. Although small, it is operationally important because several communication namespaces reuse the non-REPL interface, so moving the console between USB and UART determines which transport remains available for scripting.

## Functions

- `sentai.console()`: Returns the current REPL target, either `usb` or `uart`.
- `sentai.console('usb')`: Moves the REPL to USB CDC ACM.
- `sentai.console('uart')`: Moves the REPL to the UART console.

## Example

```python
import sentai

print(sentai.console())
sentai.console('uart')
sentai.usb.open()
sentai.usb.write(b'host channel ready\n')
sentai.usb.close()
```