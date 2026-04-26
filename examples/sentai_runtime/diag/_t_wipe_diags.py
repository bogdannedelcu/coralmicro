# _t_wipe_diags.py — recursive removal of /diags/ contents.
import sentai


def _rm(p):
    try:
        items = list(sentai.fs.ls(p))
    except Exception:
        return
    for x in items:
        n, k, _ = x
        q = p + "/" + n
        if k == 2:
            _rm(q)
        else:
            try:
                sentai.fs.remove(q)
            except Exception:
                pass
    try:
        sentai.fs.remove(p)
    except Exception:
        pass


print("=== wipe /diags ===")
_rm("/diags")
try:
    leftovers = list(sentai.fs.ls("/diags"))
except Exception:
    leftovers = None
print("after wipe, /diags:", leftovers)
print("=== done ===")
