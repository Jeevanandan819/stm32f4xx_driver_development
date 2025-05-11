PROJECT_NAME = stm32f4xx_driver_development
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJSIZE = arm-none-eabi-size

BUILD_DIR = make_build
OUTPUT_DIR = $(BUILD_DIR)/output

SRC_DIRS = . Src mcu/core/chip/src mcu/drivers/peripheral/src
INC_DIRS = . mcu/core/chip/inc mcu/drivers/peripheral/inc mcu/core/common/inc

CPU_PARAMS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
FLAGS = -fdata-sections -ffunction-sections
DEFINES = -DDEBUG
CFLAGS = $(FLAGS) -Wall -Wextra -Wno-unused-parameter -Og -g3 -ggdb -MD -std=gnu11 $(CPU_PARAMS) $(DEFINES)

STARTUP_FILE_PATH = Startup/startup_stm32f411retx.s
LINKER_SCRIPT_SRC = stm32f411retx_FLASH.ld

LDFLAGS = -T$(LINKER_SCRIPT_SRC) $(CPU_PARAMS) -Wl,-Map=$(OUTPUT_DIR)/$(PROJECT_NAME).map \
  -Wl,--gc-sections \
  -u _printf_float \
  --specs=nano.specs \
  -Wl,--start-group \
  -lc \
  -lm \
  -lstdc++ \
  -lsupc++ \
  -Wl,--end-group \
  -Wl,-z,max-page-size=8 \
  -nostdlib \
  -Wl,--print-memory-usage \
  -nostartfiles

TARGET_ELF = $(OUTPUT_DIR)/$(PROJECT_NAME).elf
TARGET_HEX = $(OUTPUT_DIR)/$(PROJECT_NAME).hex
TARGET_BIN = $(OUTPUT_DIR)/$(PROJECT_NAME).bin

# Include directories
INCLUDES = $(foreach dir,$(INC_DIRS),-I$(dir))

# Find all .c source files
C_SOURCES = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))

# Object files
C_OBJECTS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(C_SOURCES))
S_OBJECT = $(BUILD_DIR)/startup.o

# Dependency files
DEPS = $(C_OBJECTS:.o=.d)

# Final object list
OBJECTS = $(C_OBJECTS) $(S_OBJECT)

# Default target
all: $(TARGET_ELF) $(TARGET_HEX) $(TARGET_BIN)
	$(OBJSIZE) $<

# Link the final ELF file
$(TARGET_ELF): $(OBJECTS)
	@mkdir -p $(OUTPUT_DIR)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

# Generate .hex from .elf
$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex $< $@

# Generate .bin from .elf
$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

# Compile .c files
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile the startup .s file
$(S_OBJECT): $(STARTUP_FILE_PATH)
	@mkdir -p $(dir $@)
	$(CC) -c $< -o $@ $(CPU_PARAMS)

# Clean
clean:
	rm -rf $(BUILD_DIR) $(TARGET_ELF) $(TARGET_HEX) $(TARGET_BIN) $(PROJECT_NAME).map

# Include dependency files if they exist
-include $(DEPS)
