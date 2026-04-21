import sentai, gc
# Keep verbose=1 for this first shakedown per agent.md §2.7.
sentai.camera.ratio(0, 0)
sentai.camera.set_resolution(512, 512)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.switch_drain(1)

def bench(n):
    dts = []
    for i in range(n):
        t0 = sentai.rtos.ticks_ms()
        sentai.camera.select(i % 2)
        sentai.camera.jpeg(60)
        dts.append(sentai.rtos.ticks_ms() - t0)
    return dts

# warmup both cams
sentai.camera.select(0); sentai.camera.jpeg(60)
sentai.camera.select(1); sentai.camera.jpeg(60)

sentai.camera.switch_sync(0)
d0 = bench(8)
print("SYNC=0 ms:", d0, "mean=", sum(d0)//len(d0))

sentai.camera.switch_sync(1)
d1 = bench(8)
print("SYNC=1 ms:", d1, "mean=", sum(d1)//len(d1))

print("STATS:", sentai.diag.cam_stats())
