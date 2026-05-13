#!/bin/bash
nohup /home/bogdan/work/coralmicro/build-sim/sim/gz_to_uds_bridge --topic /downward_cam/image     --in-sock /tmp/sentai_cam.sock     --out-sock /tmp/sentai_flow_out.sock     > /home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/s112_x86_anchor_shim/run_cf2_20260512_204426/bridge.log 2>&1 &
disown
