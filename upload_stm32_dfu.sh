#!/usr/bin/env bash

# upload_stm32_dfu.sh
# Automates compiling and uploading custom Firmata sketches to STM32 via arduino-cli in DFU mode.
# Supports automatic fallback to raw dfu-util for Raspberry Pi (ARM64) compatibility.

SKETCH_PATH="firmware/StandardFirmataCustom_STM32/StandardFirmataCustom_STM32.ino"
FQBN="STMicroelectronics:stm32:GenF4:pnum=GENERIC_F401RCTX,upload_method=dfuMethod"
BUILD_DIR="build"

# Color constants
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== STM32 DFU arduino-cli Compilation & Upload Script ===${NC}"
echo -e "Target Sketch: ${YELLOW}${SKETCH_PATH}${NC}"
echo -e "FQBN: ${YELLOW}${FQBN}${NC}"
echo

# 1. Compile the sketch
echo -e "${BLUE}[1/2] Compiling sketch...${NC}"
mkdir -p "${BUILD_DIR}"
arduino-cli compile --fqbn "${FQBN}" --output-dir "${BUILD_DIR}" "${SKETCH_PATH}"

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Compilation failed!${NC}"
    exit 1
fi
echo -e "${GREEN}Compilation successful! Binary stored in ${BUILD_DIR}/${NC}"
echo

# 2. Instruct user to enter DFU mode
echo -e "${YELLOW}================================================================${NC}"
echo -e "${YELLOW}Please connect the STM32 board in DFU mode:${NC}"
echo -e " 1. Press and hold the ${BLUE}BOOT0${NC} button."
echo -e " 2. Press and release the ${BLUE}RESET${NC} button (or plug in the USB cable)."
echo -e " 3. Release the ${BLUE}BOOT0${NC} button."
echo -e " 4. Verify dfu-util sees the board by running: dfu-util -l"
echo -e "${YELLOW}================================================================${NC}"
echo
read -p "Press [Enter] when the board is ready in DFU mode..."

# 3. Upload the sketch
echo -e "${BLUE}[2/2] Attempting upload via arduino-cli...${NC}"
arduino-cli upload --fqbn "${FQBN}" --input-dir "${BUILD_DIR}"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}SUCCESS: Custom Firmata uploaded successfully to STM32!${NC}"
    exit 0
fi

# Fallback: If arduino-cli upload fails, it is usually because STM32CubeProgrammer is missing (especially on ARM64 Raspberry Pi)
echo -e "${YELLOW}arduino-cli upload failed. Checking for dfu-util fallback...${NC}"

if ! command -v dfu-util &> /dev/null; then
    echo -e "${RED}Error: dfu-util is not installed.${NC}"
    echo -e "To install it on Raspberry Pi / Debian, run:"
    echo -e "${BLUE}  sudo apt-get update && sudo apt-get install dfu-util${NC}"
    exit 1
fi

echo -e "${BLUE}Found dfu-util. Flashing binary directly...${NC}"
# 0483:df11 is the default STMicroelectronics bootloader DFU ID
# 0x08000000 is the flash start address for STM32F4 series
dfu-util -a 0 -d 0483:df11 --dfuse-address 0x08000000:leave -D "${BUILD_DIR}/StandardFirmataCustom_STM32.ino.bin"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}SUCCESS: Custom Firmata flashed successfully using dfu-util fallback!${NC}"
else
    echo -e "${RED}Error: dfu-util flash failed. Make sure the board is in DFU mode and try again.${NC}"
    exit 1
fi
