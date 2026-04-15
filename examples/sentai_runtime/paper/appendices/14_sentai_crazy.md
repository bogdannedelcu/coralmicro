# Appendix: sentai.crazy

The `sentai.crazy` namespace connects the runtime to a Crazyflie platform over CPX/CRTP. It exposes both higher-level flight procedures and lower-level control interfaces, allowing scripted indoor flight experiments without moving mission logic outside the main MicroPython environment.

## Functions

- `sentai.crazy.init(baud)`: Starts the Crazyflie communication tasks.
- `sentai.crazy.stop()`: Stops the bridge and closes the link.
- `sentai.crazy.debug(level)`: Changes debug verbosity.
- `sentai.crazy.arm()`: Arms the drone.
- `sentai.crazy.disarm()`: Disarms the drone.
- `sentai.crazy.ping(timeout)`: Measures round-trip time to the drone.
- `sentai.crazy.fly(height, hold_ms, takeoff_ms, land_ms)`: Executes a blocking high-level flight cycle.
- `sentai.crazy.attitude(roll, pitch, yaw_rate, thrust)`: Sends non-blocking attitude setpoints.
- `sentai.crazy.fly_stop()`: Stops attitude-controlled flight and disarms.
- `sentai.crazy.takeoff(h, dur, yaw, use_yaw, group)`: Sends a high-level takeoff command.
- `sentai.crazy.land(h, dur, yaw, use_yaw, group)`: Sends a high-level landing command.
- `sentai.crazy.stop_motors(group)`: Performs an emergency motor stop.
- `sentai.crazy.go_to(x, y, z, yaw, dur, rel, lin, group)`: Sends a waypoint-style movement command.
- `sentai.crazy.hover(vx, vy, yr, z)`: Sends hover-mode velocity commands.
- `sentai.crazy.test_fly(power, dur_ms)`: Runs a raw motor test.
- `sentai.crazy.send_crtp(port, ch, data)`: Sends an arbitrary CRTP packet.

## Example

```python
import sentai

sentai.crazy.init()
sentai.crazy.arm()
sentai.crazy.takeoff(0.5, 2.0)
sentai.rtos.sleep_ms(3000)
sentai.crazy.land(0.0, 2.0)
sentai.crazy.stop()
```