# Appendix: sentai.mesh

The `sentai.mesh` namespace bridges the runtime to Meshtastic-compatible radios over UART. It supports both simple text exchange and structured vision-message transport, making it suitable for distributed sensing, low-bandwidth telemetry, and multi-node experiments in which detections or updates must be serialized and disseminated over a mesh network.

## Functions

- `sentai.mesh.init(baud)`: Opens the UART link and starts the mesh receive task.
- `sentai.mesh.stop()`: Stops the receive task and closes the link.
- `sentai.mesh.node()`: Returns the local node number.
- `sentai.mesh.config()`: Requests configuration and NodeDB information from the radio.
- `sentai.mesh.send(text, dest, ch, ack)`: Sends a text message.
- `sentai.mesh.available()`: Returns the number of queued text messages.
- `sentai.mesh.receive(timeout)`: Receives the next text message.
- `sentai.mesh.send_detection(...)`: Sends a structured new-detection message.
- `sentai.mesh.send_update(...)`: Sends a structured track-update message.
- `sentai.mesh.vision_available()`: Returns the number of queued vision messages.
- `sentai.mesh.receive_vision(timeout)`: Receives the next structured vision message.
- `sentai.mesh.set_pose(pitch_deg, roll_deg, altitude_cm, heading_deg)`: Attaches camera pose metadata to outgoing vision messages.

## Example

```python
import sentai

sentai.mesh.init()
sentai.mesh.send('hello world')
msg = sentai.mesh.receive(5000)
if msg:
    print(msg['from'], msg['text'])
```