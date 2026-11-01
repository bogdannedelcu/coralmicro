#!/usr/bin/env python3
"""Fullscreen time code for EXP-s266: a stack of full-width horizontal stripes redrawn at the
display's frame clock.  Top stripe white (marker), 12 Gray-code stripes with the frame counter
(white = 1), bottom stripe black (marker).  Each drawn frame is logged as
`counter,monotonic_s` to --log.  Runs on the local X display of the Wayland session (XWayland),
like show_pattern.py; stop with SIGTERM / --quit.

Usage: python3 timecode_display.py [--log iter1/display_log.csv] [--bits 12] [--quit]
"""
import argparse
import glob
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PIDFILE = os.path.join(HERE, '.timecode.pid')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default=os.path.join(HERE, 'iter1', 'display_log.csv'))
    ap.add_argument('--bits', type=int, default=10)
    ap.add_argument('--y0', type=float, default=46 / 1080, help='top of the stripe stack, fraction of screen height')
    ap.add_argument('--y1', type=float, default=969 / 1080, help='bottom of the stripe stack, fraction of screen height')
    ap.add_argument('--quit', action='store_true')
    a = ap.parse_args()
    if a.quit:
        try:
            os.kill(int(open(PIDFILE).read()), signal.SIGTERM)
        except Exception:
            pass
        return
    os.makedirs(os.path.dirname(a.log), exist_ok=True)
    os.environ.setdefault('XDG_RUNTIME_DIR', '/run/user/%d' % os.getuid())
    os.environ.setdefault('DBUS_SESSION_BUS_ADDRESS', 'unix:path=%s/bus' % os.environ['XDG_RUNTIME_DIR'])
    os.environ['GDK_BACKEND'] = 'x11'
    os.environ['DISPLAY'] = ':0'
    auth = sorted(glob.glob('%s/.mutter-Xwaylandauth.*' % os.environ['XDG_RUNTIME_DIR']), key=os.path.getmtime)
    if auth:
        os.environ['XAUTHORITY'] = auth[-1]
    import gi
    gi.require_version('Gtk', '3.0')
    gi.require_version('Gdk', '3.0')
    from gi.repository import Gtk, Gdk, GdkPixbuf, GLib
    win = Gtk.Window(title='s266 timecode')
    win.set_decorated(False)
    image = Gtk.Image()
    win.add(image)
    state = {'n': 0, 'log': open(a.log, 'w'), 'size': None}
    state['log'].write('counter,t_mono\n')
    nb = a.bits
    rows = nb + 2

    def render(n):
        # 1 x rows pixbuf (marker white, nb Gray-code stripes MSB first, marker black), scaled
        # to the monitor with nearest-neighbour: no Cairo needed (python3-gi-cairo is absent)
        gray = n ^ (n >> 1)
        px = bytearray()
        for i in range(rows):
            if i == 0:
                v = 255
            elif i == rows - 1:
                v = 0
            else:
                v = 255 if (gray >> (nb - i)) & 1 else 0
            px += bytes((v, v, v))
        pb = GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(bytes(px)), GdkPixbuf.Colorspace.RGB, False, 8, 1, rows, 3)
        w, h = state['size']
        y0, y1 = int(a.y0 * h), int(a.y1 * h)
        full = GdkPixbuf.Pixbuf.new(GdkPixbuf.Colorspace.RGB, False, 8, w, h)
        full.fill(0x000000ff)                                  # black outside the stack
        pb.scale_simple(w, y1 - y0, GdkPixbuf.InterpType.NEAREST).copy_area(0, 0, w, y1 - y0, full, 0, y0)
        return full

    def tick(widget, clock):
        if state['size'] is None:
            geo = Gdk.Display.get_default().get_monitor(0).get_geometry()
            state['size'] = (geo.width, geo.height)
        state['n'] = (state['n'] + 1) % (1 << nb)
        image.set_from_pixbuf(render(state['n']))
        state['log'].write('%d,%.6f\n' % (state['n'], time.monotonic()))
        return True

    image.add_tick_callback(tick)
    win.connect('destroy', Gtk.main_quit)
    win.show_all()
    win.fullscreen_on_monitor(Gdk.Screen.get_default(), 0)
    open(PIDFILE, 'w').write(str(os.getpid()))

    def stop(*_):
        state['log'].flush()
        Gtk.main_quit()
    signal.signal(signal.SIGTERM, stop)
    Gtk.main()
    state['log'].close()


if __name__ == '__main__':
    main()
