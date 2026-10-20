# Go1 RTES project.
#   make            build everything
#   make DEBUG=1    unoptimised build with the thread sanitizer
#   make clean

CC    := gcc
WARN  := -Wall -Wextra -Wpedantic -Wshadow -Wconversion

# Source tree. main.c sits at the root; everything else is grouped by role:
#   core/     time, scheduling, blackboard, shared constants and plain data
#   logic/    control and model: kinematics, gait, attitude, simulator, render
#   threads/  the periodic tasks
# Every directory goes on the include path, so headers stay included by bare
# name and a file can move between groups without touching a single #include.
SRC_DIRS := src src/core src/logic src/threads

# rpath is baked in so the binaries run without LD_LIBRARY_PATH
MJ_DIR := mujoco-3.10.0
MJ_INC := -I$(MJ_DIR)/include
MJ_LIB := -L$(MJ_DIR)/lib -lmujoco -Wl,-rpath,$(abspath $(MJ_DIR)/lib)

# -MMD -MP emits a .d file per object listing the headers it used, so a
# change to a header rebuilds everything that includes it. Without this a
# struct can change size while stale objects keep the old layout, and the
# result is memory corruption with no compiler warning at all.
CFLAGS := -std=gnu11 $(WARN) $(addprefix -I,$(SRC_DIRS)) $(MJ_INC) -MMD -MP
LDLIBS := $(MJ_LIB) -lglfw -lGL -lpthread -lm

ifdef DEBUG
CFLAGS  += -O0 -g -fsanitize=thread
LDFLAGS += -fsanitize=thread
else
CFLAGS  += -O2 -g
endif

BUILD := build

# main.c holds the entry point, so it is not part of the shared object set.
# wildcard is not recursive, hence one pattern per directory: adding a group
# means adding it to SRC_DIRS and nothing else.
LIB_SRC := $(filter-out src/main.c,$(foreach d,$(SRC_DIRS),$(wildcard $(d)/*.c)))
LIB_OBJ := $(patsubst src/%.c,$(BUILD)/%.o,$(LIB_SRC))

EXP_SRC := $(wildcard experiments/*.c)
EXP_BIN := $(patsubst experiments/%.c,$(BUILD)/%,$(EXP_SRC))

.PHONY: all clean run
all: $(BUILD)/go1 $(EXP_BIN)

$(BUILD):
	@mkdir -p $(BUILD)

# The object mirrors the source path (build/core/rt_time.o), so the sub
# directory has to be created before gcc writes into it.
$(BUILD)/%.o: src/%.c | $(BUILD)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

-include $(LIB_OBJ:.o=.d) $(EXP_BIN:=.d) $(BUILD)/go1.d

$(BUILD)/go1: src/main.c $(LIB_OBJ) | $(BUILD)
	$(CC) $(CFLAGS) -MF $(BUILD)/go1.d $< $(LIB_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(BUILD)/%: experiments/%.c $(LIB_OBJ) | $(BUILD)
	$(CC) $(CFLAGS) -MF $@.d $< $(LIB_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

run: $(BUILD)/go1
	./$(BUILD)/go1

clean:
	@rm -rf $(BUILD)
