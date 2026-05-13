SENTAI_MOD_DIR := $(USERMOD_DIR)

# Main module in parent (sentai_runtime) directory
SRC_USERMOD_C += $(SENTAI_MOD_DIR)/../../modsentai.c

# Include paths for sentai_mesh.h -> visionmesh.pb.h -> pb.h
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../generated
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../../../third_party/nanopb
# Stage 11 — Uber H3 headers (modsentai_places.c includes "h3api.h" which
# lives at examples/sentai_runtime/h3_gen/ as a pre-generated artifact).
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../h3_gen
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../../../third_party/h3/src/h3lib/include
