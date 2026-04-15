# Appendix: sentai.link

The `sentai.link` namespace exposes a MAVLink v2 bridge over UART. It supports message reception, heartbeat and status transmission, command dispatch, structured vision reporting, and obstacle-map generation for PX4 collision prevention. It is the primary interface for integrating the runtime with a conventional autopilot stack.

## Functions

- `sentai.link.init(baud, sysid, compid)`: Opens the MAVLink link and starts the receive task.
- `sentai.link.stop()`: Stops the link and closes the UART channel.
- `sentai.link.available()`: Returns the number of queued MAVLink messages.
- `sentai.link.receive(timeout)`: Returns the next decoded MAVLink message.
- `sentai.link.heartbeat(type)`: Sends a heartbeat.
- `sentai.link.send(text, sev)`: Sends a `STATUSTEXT` message.
- `sentai.link.command(tsys, tcomp, cmd, conf, p1..p7)`: Sends a `COMMAND_LONG` message.
- `sentai.link.send_detection(...)`: Sends a structured vision detection.
- `sentai.link.send_update(...)`: Sends a structured vision update.
- `sentai.link.send_delete(...)`: Sends a structured vision deletion event.
- `sentai.link.obstacles_from_tracker(...)`: Builds and sends a 72-bin obstacle map from active tracks.
- `sentai.link.obstacle_distance(distances72, ...)`: Sends a raw `OBSTACLE_DISTANCE` message.
- `sentai.link.obstacles_from_points(points, ...)`: Builds an obstacle map from arbitrary XY points.

## Example

```python
import sentai

sentai.pipeline.init()
sentai.pipeline.set_pose(120, -1, 0.0, 0.0)
sentai.link.init(57600)
while True:
    sentai.link.obstacles_from_tracker(800, 20)
    sentai.link.heartbeat()
    sentai.rtos.sleep_ms(100)
```