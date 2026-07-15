#!/usr/bin/env bash

# installation.sh
# Comprehensive setup script to configure STM32 uploading tools (DFU, SWD, Serial) 
# and compiler toolchains on ARM SBCs (Raspberry Pi, Jetson Nano, etc.) running various Linux distros.

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==================================================================${NC}"
echo -e "${BLUE}=== STM32 Uploading & Compilation Toolchain Installer for SBCs ===${NC}"
echo -e "${BLUE}==================================================================${NC}"
echo

# 1. Detect Package Manager
echo -e "${BLUE}[1/5] Detecting Linux Distribution and Package Manager...${NC}"
if command -v apt-get &> /dev/null; then
    PKG_MANAGER="apt"
    echo -e "Detected package manager: ${GREEN}APT (Debian/Ubuntu/Raspberry Pi OS/Jetson Linux)${NC}"
elif command -v pacman &> /dev/null; then
    PKG_MANAGER="pacman"
    echo -e "Detected package manager: ${GREEN}Pacman (Arch Linux)${NC}"
elif command -v dnf &> /dev/null; then
    PKG_MANAGER="dnf"
    echo -e "Detected package manager: ${GREEN}DNF (Fedora/RedHat)${NC}"
else
    echo -e "${RED}Error: Supported package manager (APT, Pacman, DNF) not found!${NC}"
    echo -e "Please install dfu-util, openocd, and stm32flash manually using your distro's tools."
    exit 1
fi
echo

# 2. Install Flashing Utilities (dfu-util, openocd, stm32flash)
echo -e "${BLUE}[2/5] Installing flashing utilities (dfu-util, openocd, stm32flash)...${NC}"
case $PKG_MANAGER in
    apt)
        sudo apt-get update
        sudo apt-get install -y dfu-util openocd stm32flash curl build-essential
        ;;
    pacman)
        sudo pacman -Syu --noconfirm dfu-util openocd stm32flash curl base-devel
        ;;
    dnf)
        sudo dnf check-update
        sudo dnf install -y dfu-util openocd stm32flash curl @development-tools
        ;;
esac

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to install system utilities!${NC}"
    exit 1
fi
echo -e "${GREEN}Flashing utilities installed successfully!${NC}"
echo

# 3. Configure udev rules for DFU and ST-Link programmer devices
echo -e "${BLUE}[3/5] Setting up udev rules for non-root USB flashing access...${NC}"
UDEV_RULE_FILE="/etc/udev/rules.d/45-stm32-upload.rules"

sudo bash -c "cat > ${UDEV_RULE_FILE}" << 'EOF'
# STM32 Bootloader DFU Mode (VID: 0483, PID: df11)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# ST-Link V2 (VID: 0483, PID: 3748)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# ST-Link V2.1 (VID: 0483, PID: 374b)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# ST-Link V3 (VID: 0483, PID: 374f)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev", TAG+="uaccess"
EOF

# Ensure user is in dialout and plugdev groups
sudo usermod -a -G dialout $USER
sudo groupadd plugdev &> /dev/null
sudo usermod -a -G plugdev $USER

# Reload udev daemon
echo -e "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "${GREEN}udev rules configured successfully!${NC}"
echo

# 4. Install arduino-cli if missing
echo -e "${BLUE}[4/5] Checking and installing arduino-cli...${NC}"
if ! command -v arduino-cli &> /dev/null; then
    echo -e "arduino-cli not found. Downloading and installing..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    # If installed in local directory, add to PATH or move to /usr/local/bin
    if [ -f "./bin/arduino-cli" ]; then
        sudo mv ./bin/arduino-cli /usr/local/bin/
        rm -rf ./bin
    fi
else
    echo -e "${GREEN}arduino-cli is already installed: $(arduino-cli version)${NC}"
fi
echo

# 5. Configure STM32duino Core and Firmata Library
echo -e "${BLUE}[5/5] Configuring STM32 Core and libraries for arduino-cli...${NC}"

# Initialize config file if it does not exist
arduino-cli config init --overwrite &> /dev/null

# Add STM32 board manager URL
STM32_URL="https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json"
arduino-cli config set board_manager.additional_urls "${STM32_URL}"

# Update index
echo -e "Updating board manager index..."
arduino-cli core update-index

# Install STM32 core (this downloads the GCC compiler toolchain for ARM)
echo -e "Installing STMicroelectronics:stm32 core..."
arduino-cli core install STMicroelectronics:stm32

# Install Firmata library
echo -e "Installing Firmata library..."
arduino-cli lib install "Firmata"

echo
echo -e "${GREEN}==================================================================${NC}"
echo -e "${GREEN}=== Setup Completed Successfully! ===${NC}"
echo -e "${GREEN}==================================================================${NC}"
echo -e "${YELLOW}Please restart your terminal or log out/in to apply group permissions.${NC}"
echo
