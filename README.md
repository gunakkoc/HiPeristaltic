[![DOI](https://zenodo.org/badge/946078404.svg)](https://doi.org/10.5281/zenodo.15001197)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# HiPeristaltic

HiPeristaltic is a fully open-source (hardware and software) peristaltic pump with multiple independent channels, developed in Germany at **H**elmholtz **I**nstitute Erlangen-Nürnberg for Renewable Energy/Forschungszentrum Jülich.

This repo will be part of an upcoming publication of the pump from team [High-Throughput Electrocatalysis (HTE)](https://www.hi-ern.de/en/research/electrocatalysis/high-throughput-electrochemistry).

HiPeristaltic is built using 3D printed and off-the-shelf parts, with software based on Python for the user interface and pure C for the microcontroller firmware. The total cost for the entire system is approximately 250USD. It is accurate up to 0.2% volume and comes optimized, thread-safe and error-resistant software.

HiPeristaltic comes with an additional [**SiLa2**](https://sila-standard.com/standards/) software module for digital laboratories and for Self Driving Laboratories ([SDLs](https://doi.org/10.1021/acs.chemrev.4c00055)). This allows even **browser based GUI** control of the pump for non-coder users. **Python API** is also available under `/interface/`.

For the stand-alone task based stepper motor driver developed for this project, check out [**MultiStepperLite**](https://github.com/gunakkoc/MultiStepperLite).

## Supported MCUs

We are providing 3 firmwares to cover the most common microcontroller platforms:
1. STM32, tested with STM32G0B1 and includes support for TMC2209 stepper motor driver.
2. Arduino, tested with Arduino Uno Rev3(ATmega328P) and Arduino Mega 2560 (ATmega2560)
3. Raspberry Pi Pico, tested with Raspberry Pi Pico (RP2040) and Raspberry Pi Pico 2 (RP2350)

## Tested and Recommended
We've tested and recommend the following stack:
1. Using a board computer (i.e., a Raspberry Pi with an Ethernet port)
2. Using a 3D printer board (i.e., BIGTREETECH SKR Mini E3 v3.0 with STM32G0B1). We highly recommend TMC2209 or similar stepper motor drivers. However, the firmware also supports cheaper ones (i.e., A4988, DRV8825)
3. [Sila2](https://sila-standard.com/standards/) interface on the network over HTTPS protocol.

Serial UART communication via GPIO pins on Rasberry Pi is recommended. Additionally, USB connection is also supported with the MCU board or 3D printer board without requiring modifications to the firmware.

## HiPeristaltic Setup Guide

### Modify and Flash the Firmware

1. Determine the suitable firmware from the ```/firmware/``` folder.
2. Modify the GPIO pin assignments if necessary
3. Flash the firmware with the MCUs instructions.

Note: Custom 3D printer board bootloaders are not supported at this point. Therefore, some STM32 based boards might require SWD debug pins and a J-Link debugger (~5USD) to flash the firmware.

### Download and Install Raspberry Pi Imager

1. Download Raspberry Pi Imager from [here](https://www.raspberrypi.com/software/).
2. Insert your SD card into your PC.
3. Select the Raspberry Pi device.
4. For Operating System, select **Raspberry Pi OS (other)**.
5. Select the latest **Raspberry Pi OS Lite (64-bit)**.
6. Select the inserted SD card.
7. Press **Next**.

#### OS Customizations:
1. **EDIT SETTINGS**
    - Under the **GENERAL** tab:
        - Check **Set hostname** → `hiernpi.local`.
        - Check **Set username and password** (example, please change) → Username: `hte`, Password: `hte2025`.
    - Under the **SERVICES** tab:
        - Enable **SSH**.
        - Select "Use password authentication".
    - Under the **OPTIONS** tab:
        - Check **Eject media when finished**.
        - Uncheck **Enable telemetry**.

2. Press **Save**.
3. Press **Yes** to apply OS customization settings.
4. Press **Yes** to continue.

5. Once finished, remove the SD card and insert it into the Raspberry Pi.

---

### For Windows 10 and 11:

1. Install OpenSSH on Windows using the following Administrative PowerShell command:
    - Press **Windows + X** and open **Windows PowerShell (Admin)**.
    - Run the following command:
    ```bash
    Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
    ```
2. Download and install **Git for Windows** from [https://gitforwindows.org/](https://gitforwindows.org/).
3. Open **Git Bash**.

### For Linux:

1. Open terminal.
2. Install **Git**:
    ```bash
    sudo apt install git
    ```

---

### Start Raspberry Pi:

1. Plug in the Ethernet cable (or ignore if WiFi was configured).

2. Power on the Raspberry Pi. Wait approximately **5 minutes** for the first boot-up.

3. Using the console, connect to the Raspberry Pi via SSH:
    ```bash
    ssh hte@hiernpi.local
    ```
    - If asked, type **yes** and press **Enter** to continue connecting.
    - Enter the password when prompted.

4. Enable UART
   ```bash
   sudo raspi-config nonint do_serial_hw 0
   sudo raspi-config nonint do_serial_cons 1
   ```
---

### Install Mini-Forge and Dependencies:

1. Download **mini-forge**:
    ```bash
    curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
    ```

2. Install **mini-forge**:
    ```bash
    bash Miniforge3-$(uname)-$(uname -m).sh -b
    ```

3. Finalize **mini-forge** installation:
    ```bash
    ~/miniforge3/bin/conda init
    source ~/.bashrc
    ```

    The terminal should now show `(base)` as the prefix.

4. Create a new environment with Python 3.9:
    ```bash
    conda create -n hiperis python=3.9
    ```

    Type **y** and press **Enter** to proceed.

5. Activate the environment:
    ```bash
    conda activate hiperis
    ```

6. Ensure **pip** is installed:
    ```bash
    conda install pip
    ```

7. Install necessary libraries:
    ```bash
    pip install numpy pyserial toml sila2 sila2[codegen] sila2[cryptography]
    ```

8. Install **Git**:
    ```bash
    sudo apt install git
    ```

---

### Download HiPeristaltic:

1. Download **HiPeristaltic SiLa2/Python Interface**:
    ```bash
    curl -L -O "https://github.com/gunakkoc/HiPeristaltic/raw/refs/heads/main/silav2/HiPeristaltic.zip"
    ```

2. Unzip to the home directory:
    ```bash
    unzip HiPeristaltic.zip -d ~
    ```

---

### Set up the Startup Script:

1. Create a new `.sh` file to be executed on startup:
    ```bash
    nano ~/run_hiperistaltic.sh
    ```

2. Type the following inside the file:
    ```bash
    #!/bin/bash
    # Get the first IPv4 address (excluding loopback)
    CA_PATH=~/HiPeristaltic/HiPeristaltic_CA.pem
    IP_ADDR=$(hostname -I | awk '{print $1}')
    source ~/miniforge3/etc/profile.d/conda.sh
    conda activate hiperis
    cd ~/HiPeristaltic
    python -m HiPeristaltic --ip-address "$IP_ADDR" --port 50052 --ca-export-file "$CA_PATH"
    ```

3. Make the script executable:
    ```bash
    sudo chmod +x ~/run_hiperistaltic.sh
    ```

---

### Add systemd Service:

1. Create a systemd service file:
    ```bash
    sudo nano /etc/systemd/system/hiperistaltic.service
    ```

2. Type the following inside the file:
    ```bash
    [Unit]
    Description=Run HiPeristaltic Sila2 Server
    After=network.target

    [Service]
    WorkingDirectory=/home/hte/
    ExecStart=/bin/bash /home/hte/run_hiperistaltic.sh
    Restart=always

    [Install]
    WantedBy=multi-user.target
    ```

3. Enable the service to start on boot:
    ```bash
    sudo systemctl enable hiperistaltic
    ```
	
4. Reboot:
    ```bash
    sudo reboot
    ```
	
---

## Usage

### SiLa Client Over Network
- Use SiLa client.
  
or

- For a browser based Graphical User Interface(GUI), run [SiLa Universal Client](https://gitlab.com/SiLA2/universal-sila-client/sila_universal_client). Then access https://127.0.0.1:8080 . The HiPeristaltic pumps within the same networks will then be discovered.

### Serial with USB

Edit the configuration file ("~/HiPeristaltic/feature_implementations/HiPeristaltic.toml") and change accordingly (example):
	```
	"serial_port" =  "tty/USB0"
	```

### Configuration File

The default configuration file is "HiPeristaltic.toml", and by default within the same folder as "HiPeristalticInterface.py". Usually, no modifications to this file are necessary. The calibration factor can be accessed under ```calibration_uL_per_Rev``` for each pump.
