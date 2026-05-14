SENTAI_MOD_DIR := $(USERMOD_DIR)

# Main module in parent (sentai_runtime) directory
SRC_USERMOD_C += $(SENTAI_MOD_DIR)/../../modsentai.c

# Include paths for sentai_mesh.h -> visionmesh.pb.h -> pb.h
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../generated
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../../../third_party/nanopb

# ObjectsPlan L3 — sentai.places needs h3api.h (Uber H3 v4.4.1).
# CMake also configures this same header into the build dir; the static
# copy under h3_gen/ exists so the QSTR pre-pass (driven by this Makefile,
# not CMake) can find it.  Regenerate via `s119_h3_smoke/build_and_run.sh`
# logic if the H3 submodule pin is bumped.
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../h3_gen
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../../../third_party/h3/src/h3lib/include
