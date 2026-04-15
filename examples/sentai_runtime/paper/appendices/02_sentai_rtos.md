# Appendix: sentai.rtos

The `sentai.rtos` namespace exposes runtime observability and timing services from the FreeRTOS layer. It is useful both for interactive diagnosis and for scripts that need coarse scheduling, uptime measurement, or visibility into task and memory behavior during long-running onboard experiments.

## Functions

- `sentai.rtos.sleep_ms(ms)`: Suspends the current script for a specified number of milliseconds.
- `sentai.rtos.ticks_ms()`: Returns a monotonic millisecond counter.
- `sentai.rtos.tasks()`: Returns the current task list with state, priority, and stack high-water mark.
- `sentai.rtos.heap_info()`: Returns memory statistics for the RTOS heap and MicroPython GC heap.
- `sentai.rtos.cpu_usage()`: Returns per-task CPU-usage estimates.
- `sentai.rtos.uptime()`: Returns system uptime in seconds.

## Example

```python
import sentai

print('uptime:', sentai.rtos.uptime())
print('heap:', sentai.rtos.heap_info())
for name, state, prio, hwm in sentai.rtos.tasks():
    print(name, state, prio, hwm)
```