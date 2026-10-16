CC      = gcc
FW_DIR ?= ../../drone-f405-foxeer-firmware
BUILD   = build

CFLAGS  = -std=gnu11 -O2 -Wall -Wextra -g -MMD -MP
LDFLAGS = -lm
TARGET  = firedrone_sim

# simulator
SIM_SRCS = src/sim.c src/rng.c src/physics.c src/environment.c src/sensors.c \
           src/scenario.c src/pilot.c src/logger.c src/fw_host.c

# firmware: compiled unmodified, exactly the files that go into the STM32 image
FW_SRCS  = $(FW_DIR)/Allocation/cam.c $(FW_DIR)/Allocation/motor.c \
           $(FW_DIR)/Control/adrc.c $(FW_DIR)/Control/indi.c $(FW_DIR)/Control/mrac.c \
           $(FW_DIR)/Estimation/ukf.c $(FW_DIR)/Estimation/sensor_gate.c \
           $(FW_DIR)/Math/matrix.c $(FW_DIR)/Math/quaternion.c \
           $(FW_DIR)/Core/Src/crsf.c $(FW_DIR)/Core/Src/fault_manager.c \
           $(FW_DIR)/Core/Src/scheduler.c

SHIM_SRCS = shim/stm32_shim.c

FW_INC  = -Ishim -Isrc \
          -I$(FW_DIR)/Core/Inc -I$(FW_DIR)/Core/Src -I$(FW_DIR)/Drivers \
          -I$(FW_DIR)/Allocation -I$(FW_DIR)/Control -I$(FW_DIR)/Estimation \
          -I$(FW_DIR)/Math

OBJS = $(patsubst %.c,$(BUILD)/%.o,$(SIM_SRCS)) \
       $(patsubst %.c,$(BUILD)/fw/%.o,$(notdir $(FW_SRCS))) \
       $(patsubst %.c,$(BUILD)/%.o,$(SHIM_SRCS))

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD)/src/fw_host.o: src/fw_host.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -Wno-unused-parameter $(FW_INC) -c -o $@ $<

$(BUILD)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/shim/%.o: shim/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(FW_INC) -c -o $@ $<

# firmware sources live in several directories; resolve by basename
vpath %.c $(sort $(dir $(FW_SRCS)))
$(BUILD)/fw/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -Wno-unused-parameter $(FW_INC) -c -o $@ $<

-include $(OBJS:.o=.d)

# ---- original (pre-patch) firmware: scratch copy of FW_DIR with baseline/ files laid over it ----
ORIG_FW = build/fw_orig
BASELINE = $(shell find baseline -type f 2>/dev/null)

$(ORIG_FW)/.stamp: $(BASELINE)
	rm -rf $(ORIG_FW)
	mkdir -p $(ORIG_FW)/Drivers
	cp -r $(FW_DIR)/Allocation $(FW_DIR)/Control $(FW_DIR)/Estimation \
	      $(FW_DIR)/Math $(FW_DIR)/Core $(ORIG_FW)/
	cp $(FW_DIR)/Drivers/*.h $(ORIG_FW)/Drivers/
	cp -r baseline/. $(ORIG_FW)/
	touch $@

ifndef IN_ORIG
orig: $(ORIG_FW)/.stamp
	$(MAKE) --no-print-directory IN_ORIG=1 FW_DIR=$(ORIG_FW) \
	        BUILD=build/orig TARGET=firedrone_sim_orig

all: $(TARGET) orig
endif

clean:
	rm -rf build $(TARGET) firedrone_sim_orig firedrone_x8_sim

distclean: clean
	rm -rf out
	$(MAKE) -C timing clean

.PHONY: clean distclean all orig

# X8 design study: the same C99 control sources also compile for Cortex-M7.
# This is a portable control library, not a board-specific H7 flash image.
X8_SRCS = src/x8_sim.c src/x8_config.c src/physics.c src/environment.c src/rng.c \
          src/sensors.c src/scenario.c src/logger.c flight_core/mission.c \
          flight_core/allocator.c flight_core/control.c \
          $(FW_DIR)/Estimation/ukf.c $(FW_DIR)/Estimation/sensor_gate.c \
          $(FW_DIR)/Math/matrix.c $(FW_DIR)/Math/quaternion.c \
          $(FW_DIR)/Control/adrc.c $(FW_DIR)/Control/mrac.c
X8_INC = -Iflight_core -Isrc -I$(FW_DIR)/Estimation -I$(FW_DIR)/Control -I$(FW_DIR)/Math
firedrone_x8_sim: $(X8_SRCS) $(wildcard flight_core/*.h src/*.h $(FW_DIR)/Control/*.h $(FW_DIR)/Estimation/*.h $(FW_DIR)/Math/*.h)
	$(CC) -std=c99 -D_DEFAULT_SOURCE -O2 -Wall -Wextra $(X8_INC) $(X8_SRCS) -lm -o $@

x8: firedrone_x8_sim

h7-core:
	@mkdir -p build/h7-core
	@set -e; for src in flight_core/allocator.c flight_core/control.c flight_core/mission.c \
	  $(FW_DIR)/Estimation/ukf.c $(FW_DIR)/Estimation/sensor_gate.c \
	  $(FW_DIR)/Math/matrix.c $(FW_DIR)/Math/quaternion.c \
	  $(FW_DIR)/Control/adrc.c $(FW_DIR)/Control/mrac.c; do \
	  arm-none-eabi-gcc -std=c99 -D_DEFAULT_SOURCE -O2 -Wall -Wextra \
	    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard $(X8_INC) \
	    -c "$$src" -o "build/h7-core/$$(basename "$$src" .c).o"; done
	arm-none-eabi-ar rcs build/h7-core/libfiredrone.a build/h7-core/*.o

.PHONY: x8 h7-core

test-x8:
	@mkdir -p build/tests
	$(CC) -std=c99 -D_DEFAULT_SOURCE -O1 -g -Wall -Wextra \
	  $(SANITIZE) -fno-omit-frame-pointer $(X8_INC) \
	  tests/test_x8.c $(filter-out src/x8_sim.c src/logger.c,$(X8_SRCS)) \
	  -lm -o build/tests/test_x8
	./build/tests/test_x8

.PHONY: test-x8


build/libfiredrone_live.so: src/live_sim.c src/live_sim.h $(X8_SRCS) $(wildcard flight_core/*.h src/*.h $(FW_DIR)/Control/*.h $(FW_DIR)/Estimation/*.h $(FW_DIR)/Math/*.h)
	@mkdir -p build
	$(CC) -std=c99 -D_DEFAULT_SOURCE -O2 -Wall -Wextra -fPIC -shared $(X8_INC) \
	  src/live_sim.c $(filter-out src/x8_sim.c src/logger.c,$(X8_SRCS)) -lm -o $@.tmp
	mv $@.tmp $@

live: build/libfiredrone_live.so
.PHONY: live

test-live:
	@mkdir -p build/tests
	$(CC) -std=c99 -D_DEFAULT_SOURCE -O1 -g -Wall -Wextra $(SANITIZE) $(X8_INC) \
	  tests/test_live.c $(filter-out src/x8_sim.c src/logger.c,$(X8_SRCS)) -lm -o build/tests/test_live
	./build/tests/test_live

.PHONY: test-live
