// MicroPython FreeRTOS task wrapper
// Runs a MicroPython script or interactive REPL in a dedicated FreeRTOS task

#ifndef MICROPYTHON_TASK_H_
#define MICROPYTHON_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

// Start a FreeRTOS task that runs the given Python script string.
// script: null-terminated Python source code
// stack_size: FreeRTOS task stack size in bytes
// priority: FreeRTOS task priority
void micropython_start_task(const char* script, unsigned int stack_size,
                            unsigned int priority);

// Start a FreeRTOS task with an interactive MicroPython REPL.
// Reads Python commands from the serial console (same as printf),
// executes them, and prints results.
// stack_size: FreeRTOS task stack size in bytes
// priority: FreeRTOS task priority
void micropython_start_repl_task(unsigned int stack_size,
                                 unsigned int priority);

#ifdef __cplusplus
}
#endif

#endif  // MICROPYTHON_TASK_H_
