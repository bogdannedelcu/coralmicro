# Appendix: sentai.usb

The `sentai.usb` namespace provides two distinct services: mass-storage export of the user partition and raw USB CDC serial I/O when the REPL has been moved away from USB. It is therefore both a deployment path for files and a host-device communication channel, with explicit coordination required to avoid conflicts with filesystem access.

## Functions

- `sentai.usb.drive(on)`: Enables or disables USB mass-storage export.
- `sentai.usb.open()`: Opens USB CDC ACM for raw serial I/O.
- `sentai.usb.close()`: Closes the raw USB CDC channel.
- `sentai.usb.write(data)`: Writes bytes or text to the USB host.
- `sentai.usb.read(max, timeout_ms)`: Reads bytes from the USB host.
- `sentai.usb.available()`: Returns the number of pending received bytes.

## Example

```python
import sentai

sentai.usb.drive(1)
# host copies files here
sentai.usb.drive(0)
print(sentai.fs.ls('/'))
```