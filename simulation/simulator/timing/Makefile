# Cross-compiles the real flight stages for the STM32F405 (Cortex-M4F).
FW_DIR ?= ../../../drone-f405-foxeer-firmware
OPT    ?= -Og

MCU    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS = $(MCU) $(OPT) -Wall -ffunction-sections -fdata-sections -ffreestanding
INC    = -I$(FW_DIR)/Math -I$(FW_DIR)/Estimation -I$(FW_DIR)/Control \
         -I$(FW_DIR)/Allocation -I$(FW_DIR)/Core/Inc

SRCS   = fw_bench.c $(FW_DIR)/Math/matrix.c $(FW_DIR)/Math/quaternion.c \
         $(FW_DIR)/Estimation/ukf.c $(FW_DIR)/Control/indi.c $(FW_DIR)/Control/adrc.c \
         $(FW_DIR)/Control/mrac.c $(FW_DIR)/Allocation/cam.c $(FW_DIR)/Core/Src/fault_manager.c

fw_bench.elf: $(SRCS) link.ld
	arm-none-eabi-gcc $(CFLAGS) $(INC) $(SRCS) -T link.ld -nostartfiles \
	    -specs=nano.specs -specs=nosys.specs -lm -o $@
	arm-none-eabi-objcopy -O binary $@ fw_bench.bin
	arm-none-eabi-size $@

clean:
	rm -f fw_bench.elf fw_bench.bin

.PHONY: clean
