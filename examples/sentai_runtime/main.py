# main.py — boot-time auto-init for radio-only operation.
#
# After this runs, the board can be driven entirely over Crazyradio
# (no USB cable required) via the $-prefix REPL exec path:
#   cf.send_packet(port=0x0E, channel=0, data=b'$1+1')
#   cf.send_packet(port=0x0E, channel=0, data=b'$sentai.crazy.baro()')
# Replies arrive on the same port via add_port_callback(0x0E, ...).
#
# What this does (in order):
#   1. sentai.crazy.init()   — bring up UART2 + 0xAA wire bridge so the
#                              drone deck driver can forward radio CRTP
#                              packets to/from the board's MicroPython.
#                              Required for $-prefix REPL exec to work.
#                              Sets g_crazy_running = True (without this
#                              flag, sentai.crazy.telem() returns -1).
#   2. sentai.camera.init()  — boot camera with FLOW BASELINE orientation
#                              (cam_hflip=0, cam_vflip=1 — the sentai.camera
#                              firmware default since 2026-05-08).  Both
#                              OV5640s have their 0x3820/0x3821 written
#                              accordingly.  See agent.md §18 for the
#                              empirical body-axis convention this matches.
#   3. sentai.flow.enable()  — stamp the FLOW_SHARED magic word so
#                              consumers (sentai.flow.read, etc.) don't
#                              read uninitialised data.
#   4. sentai.flow.start(0)  — spawn the publisher_task on cam0 (front,
#                              USB-side, drone forward = cam0->cam1 axis).
#                              After this, sentai.flow.read() returns
#                              live phase-correlation results @ ~30 Hz.
#
# What this does NOT do:
#   - Inject flow to the drone EKF.  That's a separate loop the user
#     starts on demand:
#       $import diag._t_flow_inject as f; f.go(secs=20)
#     (or via the REPL paste-bench from a host with USB connected).
#   - Switch console to UART.  REPL stays on USB CDC by default; if
#     the USB cable is plugged in /dev/ttyACM0 still works as a normal
#     REPL.  Single-UART mutual exclusion (agent.md §17.5) means we
#     CANNOT have console on UART AND sentai.crazy at the same time.
#     With console on USB, sentai.crazy owns the UART for radio bridge.
#   - Set the drone's Kalman estimator.  stabilizer.estimator defaults
#     to 1 (Complementary) which IGNORES our flow injections.  Set it
#     to 2 (Kalman) over radio once per boot:
#       cf.param.set_value('stabilizer.estimator', 2)
#     (Not PARAM_PERSISTENT upstream, so it resets every drone boot.)
#
# To disable auto-init (e.g. for diag work that conflicts with flow):
#   $sentai.flow.stop()                     # stop publisher, free UART RX
#   or remove this file entirely:
#   $sentai.fs.remove('/main.py')
#   then sys.reset() to reboot without auto-init.

import sentai

sentai.crazy.init()
sentai.camera.init()
sentai.flow.enable()
sentai.flow.start(0)
