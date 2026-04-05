SENTAI_MOD_DIR := $(USERMOD_DIR)

# Main module in parent (sentai_runtime) directory
SRC_USERMOD_C += $(SENTAI_MOD_DIR)/../../modsentai.c

# Include paths for sentai_mesh.h -> visionmesh.pb.h -> pb.h
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../generated
CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../../../../third_party/nanopb
