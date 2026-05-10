#!/bin/bash
# Yaw compensation test:
#   - Drone takes off with flow injection enabled
#   - Hovers at 1m for 8s (settle X/Y)
#   - Yaws +180 deg over 4s (yawrate ~0.78 rad/s)
#   - Wind continues to push +Y world
#   - Check: does drone stay near (0,0) despite the rotation?
#     Body-frame flow should still correctly compensate world-frame wind
#     after EKF applies the new attitude rotation R(yaw) to body velocities.
#
# Pass criterion: peak drift in world-frame Y < 0.5 m during the rotation
# (vs >1m baseline drift over equivalent time).

cd /home/bogdan/work/coralmicro
source /home/bogdan/work/crazyflie/.venv/bin/activate
export PYTHONWARNINGS=ignore
OUT=/tmp/yaw_compensation_pose.csv
echo "t_s,x,y,z,yaw_deg,sent" > "$OUT"

python3 - <<PY 2>/dev/null
import time, threading, socket, struct, math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

OUT="$OUT"
CFG=dict(fov_h_deg=58.0, fov_v_deg=45.0, grid_w=80, grid_h=60, drone_npix=35.0,
         drone_thetapix_rad=0.71674, drone_flow_resolution=0.10,
         body_xform=(-1.0,0.0,0.0,+1.0))
SX=(math.radians(CFG["fov_h_deg"])*CFG["drone_npix"])/(CFG["grid_w"]*CFG["drone_flow_resolution"]*CFG["drone_thetapix_rad"])
SY=(math.radians(CFG["fov_v_deg"])*CFG["drone_npix"])/(CFG["grid_h"]*CFG["drone_flow_resolution"]*CFG["drone_thetapix_rad"])

MAX_TILT_RAD = 0.35
stop=threading.Event(); enable=threading.Event(); n_sent=[0]; n_gated=[0]
alt={"z":0.0,"roll":0.0,"pitch":0.0}
def std_scale(z):
    if z<=0.30: return 8.0
    if z>=1.0:  return 1.0
    return 1.0 + 7.0*(1.0-z)/0.7

def flow_th(cf):
    s=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(0.5)
    try: s.connect("/tmp/sentai_flow_out.sock")
    except: print("FLOW connect fail"); return
    R,RL="<IIiiIQ",struct.calcsize("<IIiiIQ"); buf=b""; tl=time.monotonic()
    while not stop.is_set():
        try: c=s.recv(RL*4)
        except socket.timeout: continue
        except OSError: break
        if not c: break
        buf+=c
        while len(buf)>=RL:
            rec,buf=buf[:RL],buf[RL:]
            m,sq,dx,dy,cf_,lat=struct.unpack(R,rec)
            if m!=0x46524C31: continue
            now=time.monotonic(); dt=max(1e-3,min(0.2,now-tl)); tl=now
            if not enable.is_set(): n_gated[0]+=1; continue
            if cf_==0: n_gated[0]+=1; continue
            if abs(alt["roll"])>MAX_TILT_RAD or abs(alt["pitch"])>MAX_TILT_RAD:
                n_gated[0]+=1; continue
            fdx,fdy,ldx,ldy=CFG["body_xform"]
            dxg=dx/1000.0; dyg=dy/1000.0
            fw=fdx*dxg+fdy*dyg; lf=ldx*dxg+ldy*dyg
            dpx=max(-32768,min(32767,int(round(fw*SX))))
            dpy=max(-32768,min(32767,int(round(lf*SY))))
            base_std=1.0 if cf_>=200 else (2.0 if cf_>=128 else 4.0)
            std=base_std*std_scale(alt["z"])
            pk=CRTPPacket(); pk.port=CRTPPort.LOCALIZATION; pk.channel=1
            pk.data=struct.pack("<fhhfHH",float(dt),dpx,dpy,float(std),0,0)
            try: cf.send_packet(pk); n_sent[0]+=1
            except: pass

cflib.crtp.init_drivers()
sync=SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
sync.open_link(); cf=sync.cf
cf.param.set_value("stabilizer.estimator",2); time.sleep(1.0)
threading.Thread(target=flow_th,args=(cf,),daemon=True).start()
time.sleep(2.0)

state={"x":0,"y":0,"z":0,"yaw":0,"t0":time.monotonic()}
log=LogConfig(name="pose",period_in_ms=100)
log.add_variable("stateEstimate.x","float"); log.add_variable("stateEstimate.y","float"); log.add_variable("stateEstimate.z","float")
log.add_variable("stateEstimate.yaw","float")
log.add_variable("stabilizer.roll","float"); log.add_variable("stabilizer.pitch","float")
def cb(ts,d,_):
    state["x"]=d["stateEstimate.x"]; state["y"]=d["stateEstimate.y"]
    state["z"]=d["stateEstimate.z"]; state["yaw"]=d["stateEstimate.yaw"]
    alt["z"]=state["z"]
    alt["roll"]=math.radians(d["stabilizer.roll"]); alt["pitch"]=math.radians(d["stabilizer.pitch"])
    if state["z"]>0.30 and not enable.is_set(): enable.set()
    t=time.monotonic()-state["t0"]
    with open(OUT,"a") as f: f.write(f"{t:.3f},{state['x']:.3f},{state['y']:.3f},{state['z']:.3f},{state['yaw']:.1f},{n_sent[0]}\n")
cf.log.add_config(log); log.data_received_cb.add_callback(cb); log.start()

print("Phase 1: ramp 0->0.3m + hold 2s + climb to 1m", flush=True)
t0=time.monotonic()
while time.monotonic()-t0<1.0: cf.commander.send_hover_setpoint(0,0,0,0.30*(time.monotonic()-t0)); time.sleep(0.05)
t0=time.monotonic()
while time.monotonic()-t0<2.0: cf.commander.send_hover_setpoint(0,0,0,0.30); time.sleep(0.05)
t0=time.monotonic()
while time.monotonic()-t0<2.0:
    frac=(time.monotonic()-t0)/2.0
    cf.commander.send_hover_setpoint(0,0,0, 0.30+(1.0-0.30)*frac); time.sleep(0.05)

print("Phase 2: stable hover 8s @ 1m (no rotation)", flush=True)
t0=time.monotonic()
while time.monotonic()-t0<8.0:
    cf.commander.send_hover_setpoint(0,0,0,1.0); time.sleep(0.05)

print("Phase 3: YAW +180 deg in 4s (yawrate ~ 45 deg/s)", flush=True)
t0=time.monotonic(); YAW_RATE=45.0  # deg/s
while time.monotonic()-t0<4.0:
    cf.commander.send_hover_setpoint(0,0,YAW_RATE,1.0); time.sleep(0.05)

print("Phase 4: stable hover 8s @ 1m (post-rotation, drone facing -X)", flush=True)
t0=time.monotonic()
while time.monotonic()-t0<8.0:
    cf.commander.send_hover_setpoint(0,0,0,1.0); time.sleep(0.05)

print("Land", flush=True)
t0=time.monotonic()
while time.monotonic()-t0<3.0:
    cf.commander.send_hover_setpoint(0,0,0,max(0.05,1.0*(1-(time.monotonic()-t0)/3))); time.sleep(0.05)
cf.commander.send_stop_setpoint(); log.stop(); stop.set(); sync.close_link()
print(f"DONE sent={n_sent[0]}", flush=True)
PY

echo ""
echo "=== analysis ==="
python3 - <<PY
import csv
rows=list(csv.reader(open("$OUT")))[1:]
T,X,Y,Z,YAW,SENT = zip(*[(float(r[0]),float(r[1]),float(r[2]),float(r[3]),float(r[4]),int(r[5])) for r in rows])
def stats(label, t_lo, t_hi):
    win=[(t,x,y,z,y_) for t,x,y,z,y_ in zip(T,X,Y,Z,YAW) if t_lo<=t<=t_hi]
    if not win: print(f"  {label}: no samples"); return
    xs=[x for _,x,*_ in win]; ys=[y for _,_,y,*_ in win]; zs=[z for _,_,_,z,_ in win]; yaws=[y_ for _,_,_,_,y_ in win]
    print(f"  {label}: x [{min(xs):+.2f}..{max(xs):+.2f}] y [{min(ys):+.2f}..{max(ys):+.2f}] z [{min(zs):+.2f}..{max(zs):+.2f}] yaw [{min(yaws):+.0f}..{max(yaws):+.0f}]")
stats("pre-rotate hover  (5..13s)  ", 5.0, 13.0)
stats("during rotation   (13..17s) ", 13.0, 17.0)
stats("post-rotate hover (17..25s) ", 17.0, 25.0)
PY
