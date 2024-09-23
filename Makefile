PROGRAM_NAME = toilet-controller
SRC_DIR = src
BUILD_DIR = build

all: $(PROGRAM_NAME)

# Create the build directory
$(BUILD_DIR):
	mkdir -pv $(BUILD_DIR)

# Compile and build the program for AtTiny85
$(PROGRAM_NAME): $(BUILD_DIR)
	avr-gcc -mmcu=attiny85 -Wall -Os -o $(BUILD_DIR)/$(PROGRAM_NAME).elf $(SRC_DIR)/$(PROGRAM_NAME).c
	avr-objcopy -j .text -j .data -O ihex $(BUILD_DIR)/$(PROGRAM_NAME).elf $(BUILD_DIR)/$(PROGRAM_NAME).hex

# Upload the built program (hex file) to Atmega328P using USBasp
#upload: $(PROGRAM_NAME)
	avrdude -c avrispmkII -p t85 -U flash:w:$(BUILD_DIR)/$(PROGRAM_NAME).hex

# Remove build directory with all built files
#clean:
#	rm -rf $(BUILD_DIR) 
