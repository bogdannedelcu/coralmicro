import sys, time
sys.path.insert(0, "/home/bogdan/work/coralmicro/examples/sentai_runtime/diag")
import sentai
print("=BOOT_OK")
print(sentai.flow.mode("anchor"))
for i in range(40):                      # ~20s @ 0.5s
    p = sentai.flow.anchor_pose()
    print("ANCHOR", i, p["detected"], p["num_markers"],
          p["x"], p["y"], p["z"], p["frame_seq"], p["src_ts_ms"])
    sentai.rtos.sleep_ms(500)
print("=DONE")
