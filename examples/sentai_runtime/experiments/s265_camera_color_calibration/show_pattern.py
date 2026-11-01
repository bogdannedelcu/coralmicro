#!/usr/bin/env python3
"""Fullscreen pattern display on the calibration monitor (GNOME/Wayland, GTK 3).

Usage:
  python3 show_pattern.py --solid 808080            # uniform grey
  python3 show_pattern.py --grid                    # 24-patch ColorChecker-like grid
  python3 show_pattern.py --png file.png            # any image, letterboxed
  python3 show_pattern.py --quit                    # stop the running instance

The window is fullscreen on the first (only) monitor of the Wayland session and
stays up until --quit.  Patch sRGB values of the grid are in PATCHES (also used by
analyze.py), laid out 6 columns x 4 rows, with a mid-grey border.  Generated
images are saved under <out>/pattern_*.png so the shown pattern is on record.
"""
import argparse
import json
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PIDFILE = os.path.join(HERE, '.show_pattern.pid')

# X-Rite ColorChecker Classic sRGB values (D65), rows top to bottom.
PATCHES = [
    ('dark skin', (115, 82, 68)), ('light skin', (194, 150, 130)), ('blue sky', (98, 122, 157)),
    ('foliage', (87, 108, 67)), ('blue flower', (133, 128, 177)), ('bluish green', (103, 189, 170)),
    ('orange', (214, 126, 44)), ('purplish blue', (80, 91, 166)), ('moderate red', (193, 90, 99)),
    ('purple', (94, 60, 108)), ('yellow green', (157, 188, 64)), ('orange yellow', (224, 163, 46)),
    ('blue', (56, 61, 150)), ('green', (70, 148, 73)), ('red', (175, 54, 60)),
    ('yellow', (231, 199, 31)), ('magenta', (187, 86, 149)), ('cyan', (8, 133, 161)),
    ('white', (243, 243, 242)), ('neutral 8', (200, 200, 200)), ('neutral 6.5', (160, 160, 160)),
    ('neutral 5', (122, 122, 121)), ('neutral 3.5', (85, 85, 85)), ('black', (52, 52, 52)),
]
COLS, ROWS = 6, 4


def grid_image(w=1920, h=1080, border=0.06, gap=0.012, bg=(118, 118, 118), region=None,
               cols=COLS, rows=ROWS):
    """24-patch grid; ``region`` = (x, y, rw, rh) screen rectangle to fill, default all.
    ``cols`` x ``rows`` must hold the 24 patches (6x4 landscape or 4x6 portrait)."""
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (w, h), bg)
    d = ImageDraw.Draw(img)
    rx, ry, rw, rh = region if region else (0, 0, w, h)
    bx, by = int(rw * border), int(rh * border)
    gw, gh = rw - 2 * bx, rh - 2 * by
    bx, by = bx + rx, by + ry
    gx, gy = int(rw * gap), int(rh * gap)
    pw = (gw - (cols - 1) * gx) // cols
    ph = (gh - (rows - 1) * gy) // rows
    rects = []
    for i, (name, rgb) in enumerate(PATCHES):
        c, r = i % cols, i // cols
        x0 = bx + c * (pw + gx)
        y0 = by + r * (ph + gy)
        d.rectangle([x0, y0, x0 + pw - 1, y0 + ph - 1], fill=rgb)
        rects.append({'name': name, 'srgb': rgb, 'rect': [x0, y0, pw, ph]})
    return img, rects


RAMP = [('g%03d' % v, (v, v, v)) for v in
        (8, 18, 28, 38, 48, 58, 68, 78, 88, 98, 108, 118, 128, 138, 148, 158, 168, 178, 190,
         202, 214, 226, 238, 250)]


def ramp_image(w=1920, h=1080, region=None, cols=COLS, rows=ROWS):
    """24-step grey ramp in the grid layout (tone-curve stage)."""
    global PATCHES
    saved, PATCHES = PATCHES, RAMP
    try:
        return grid_image(w, h, region=region, cols=cols, rows=rows)
    finally:
        PATCHES = saved


def solid_image(rgb, w=1920, h=1080):
    from PIL import Image
    return Image.new('RGB', (w, h), rgb)


def stop_running():
    if os.path.exists(PIDFILE):
        try:
            os.kill(int(open(PIDFILE).read()), signal.SIGTERM)
        except Exception:
            pass
        os.remove(PIDFILE)


CTRL = os.path.join(HERE, 'patterns', '.current_png')


def server_alive():
    try:
        pid = int(open(PIDFILE).read())
        os.kill(pid, 0)
        return pid
    except Exception:
        return 0


def show(path):
    """Hand ``path`` to the running server (it polls CTRL every 100 ms)."""
    os.makedirs(os.path.dirname(CTRL), exist_ok=True)
    open(CTRL, 'w').write(path)


def serve(path):
    """Persistent fullscreen window.  It polls CTRL every 100 ms and swaps the image when
    the file names another PNG, so a sequence of patterns needs no window restarts (an X11
    window restart took > 1 s to map and the first captures saw the desktop)."""
    os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')
    os.environ.setdefault('XDG_RUNTIME_DIR', '/run/user/%d' % os.getuid())
    os.environ.setdefault('DBUS_SESSION_BUS_ADDRESS',
                          'unix:path=%s/bus' % os.environ['XDG_RUNTIME_DIR'])
    # Run through XWayland: a native Wayland GTK fullscreen window makes GNOME Shell pop
    # up the modal "allow inhibiting shortcuts?" dialog, which dims the whole screen and
    # covers the centre of the pattern (s265 dead end, 2026-11-01).  X11 has no such protocol.
    # DISPLAY is forced: an SSH shell carries DISPLAY=localhost:10.0 (X forwarding) and the
    # window would silently open on the remote X server instead of the calibration monitor.
    import glob
    os.environ['GDK_BACKEND'] = 'x11'
    os.environ['DISPLAY'] = ':0'
    auth = glob.glob('%s/.mutter-Xwaylandauth.*' % os.environ['XDG_RUNTIME_DIR'])
    if auth:
        os.environ['XAUTHORITY'] = auth[0]
    import gi
    gi.require_version('Gtk', '3.0')
    gi.require_version('Gdk', '3.0')
    from gi.repository import Gtk, Gdk, GdkPixbuf
    win = Gtk.Window(title='s265 pattern')
    win.set_decorated(False)
    css = Gtk.CssProvider()
    css.load_from_data(b'window { background-color: #000000; }')
    Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css,
                                             Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    disp = Gdk.Display.get_default()
    geo = disp.get_monitor(0).get_geometry()
    pb = GdkPixbuf.Pixbuf.new_from_file_at_scale(path, geo.width, geo.height, True)
    image = Gtk.Image.new_from_pixbuf(pb)
    win.add(image)
    win.connect('destroy', Gtk.main_quit)
    win.show_all()
    win.fullscreen_on_monitor(Gdk.Screen.get_default(), 0)
    open(PIDFILE, 'w').write(str(os.getpid()))
    state = {'path': path}

    def poll():
        try:
            new = open(CTRL).read().strip()
        except Exception:
            return True
        if new and new != state['path'] and os.path.exists(new):
            try:
                image.set_from_pixbuf(GdkPixbuf.Pixbuf.new_from_file_at_scale(new, geo.width, geo.height, True))
                state['path'] = new
                print('showing', new, flush=True)
            except Exception as e:  # keep serving on a bad file
                print('cannot load', new, e, flush=True)
        return True

    from gi.repository import GLib
    GLib.timeout_add(100, poll)
    Gtk.main()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--solid')
    ap.add_argument('--grid', action='store_true')
    ap.add_argument('--ramp', action='store_true', help='24-step grey ramp, same layout as --grid')
    ap.add_argument('--png')
    ap.add_argument('--region', help='x,y,w,h screen rectangle for --grid (default full screen)')
    ap.add_argument('--layout', default='6x4', help='colsxrows for --grid: 6x4 or 4x6')
    ap.add_argument('--quit', action='store_true')
    ap.add_argument('--out', default=os.path.join(HERE, 'patterns'))
    ap.add_argument('--fg', action='store_true', help='run in the foreground')
    a = ap.parse_args()
    if a.quit:
        stop_running()
        return
    os.makedirs(a.out, exist_ok=True)
    if a.solid:
        rgb = tuple(int(a.solid[i:i + 2], 16) for i in (0, 2, 4))
        path = os.path.join(a.out, 'pattern_solid_%s.png' % a.solid.lower())
        solid_image(rgb).save(path)
    elif a.grid or a.ramp:
        region = tuple(int(v) for v in a.region.split(',')) if a.region else None
        cols, rows = (int(v) for v in a.layout.split('x'))
        kind = 'grid' if a.grid else 'ramp'
        img, rects = (grid_image if a.grid else ramp_image)(region=region, cols=cols, rows=rows)
        path = os.path.join(a.out, 'pattern_%s.png' % kind)
        img.save(path)
        json.dump(rects, open(os.path.join(a.out, 'pattern_%s.json' % kind), 'w'), indent=1)
    elif a.png:
        path = a.png
    else:
        ap.error('one of --solid/--grid/--png/--quit')
    show(path)                      # hand the file to a running server, if any
    if server_alive():
        print('showing %s (server pid %d)' % (path, server_alive()))
        return
    stop_running()
    if a.fg:
        serve(path)
        return
    pid = os.fork()
    if pid == 0:
        os.setsid()
        # redirect at fd level: an inherited stdout pipe would keep the caller's
        # `| tail` waiting forever (seen with shading_map.py)
        fd = os.open(os.path.join(a.out, 'show_pattern.log'), os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
        os.dup2(fd, 1)
        os.dup2(fd, 2)
        devnull = os.open(os.devnull, os.O_RDONLY)
        os.dup2(devnull, 0)
        serve(path)
        os._exit(0)
    print('showing %s (pid %d)' % (path, pid))


if __name__ == '__main__':
    main()
