import sentai, gc

def _rmtree(path):
    try:
        entries = sentai.fs.ls(path)  # list of (name, type, size); type 1=file 2=dir
    except Exception:
        sentai.fs.remove(path)
        return
    for (nm, tp, _sz) in entries:
        child = path + "/" + nm
        if tp == 2:
            _rmtree(child)
        else:
            try: sentai.fs.remove(child)
            except Exception as ex: print("  skip f", child, ex)
    try: sentai.fs.remove(path)
    except Exception as ex: print("  rmdir fail", path, ex)

top = sentai.fs.ls("/diags")
total = 0
for (nm, tp, _sz) in top:
    if tp != 2 or not nm.startswith("s"):
        continue
    p = "/diags/" + nm
    _rmtree(p)
    total += 1
    if total % 5 == 0:
        gc.collect()
        print("  cleaned %d dirs so far" % total)

print("== cleaned %d session dirs ==" % total)
print("remaining in /diags:", sentai.fs.ls("/diags"))
