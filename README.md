[![DOI](https://zenodo.org/badge/946078404.svg)](https://doi.org/10.5281/zenodo.15001197)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# HiPeristaltic

HiPeristaltic is a fully open-source (hardware and software) peristaltic pump with multiple independent channels, developed in Germany at **H**elmholtz **I**nstitute Erlangen-Nürnberg for Renewable Energy/Forschungszentrum Jülich.

This repo will be part of an upcoming publication of the pump from team [High-Throughput Electrocatalysis (HTE)](https://www.hi-ern.de/en/research/electrocatalysis/high-throughput-electrochemistry).

HiPeristaltic is built using 3D printed and off-the-shelf parts, with software based on Python for the user interface and pure C for the microcontroller firmware. The total cost for the entire system is approximately 280USD. It is accurate up to 0.2% volume and comes optimized, thread-safe and error-resistant software.

HiPeristaltic includes a [**SiLa2**](https://sila-standard.com/standards/) software module for easy integration to digital laboratories and for Self Driving Laboratories ([SDLs](https://doi.org/10.1021/acs.chemrev.4c00055)). SiLa2 also allows controlling the pump with a **GUI over web browser** for non-coder users. Additionally, **Python API** is available under `/interface/` for non-SiLa integration.

The software stack is designed to be compatible with a wide range of MCUs (STM32, Arduino, Pico), stepper motor drivers (A4988, DRV8825, TMC2209, ...) and connectivity options (UART,USB,Ethernet/Wifi).

- Users who just want to get started can refer to [Getting Started Quickly](#Getting-Started-Quickly). This is the best option to replicate HiPeristaltic as published and with the recommended hardware.
- For experienced users, a step by step guide is given in [Advanced Installation](#Advanced-Installation) to enable customizations.
- For those familiar with MCU platforms and motor drivers, all other supported hardware combinations are listed under [Hardware Options](#Hardware-Options).
- For developers looking to integrate concurrent stepper motor control, a spin-off library [**MultiStepperLite**](https://github.com/gunakkoc/MultiStepperLite) is also published.

# Getting Started Quickly

In this section, only the most straightforwad software setup to enable pump through SiLa2 is provided. The hardware connections are assumed to be completed as given in the publication. Furthermore, this tutorial assumes the user has the following:

- Raspberry Pi 4 Model B
- A microSD card with minimum 16GB capacity.
- BIGTREETECH SKR Mini E3 v3.0 board
- Another microSD card with 1MB capacity.
- Rasberry Pi 4 Model B and the client (e.g., user or laboratory PC) computers being on the same network (i.e., both are connected to the same router or same Ethernet switch)

---

1. Download and flash the Rasberry Pi image to the microSD card.
2. Insert this microSD card to Rasberry Pi but don't turn it on yet.
3. Download and place the firmware to the other SD Card.
4. Insert this SD card to BIGTREETECH SKR Mini E3 v3.0 board, then power on and wait for 15 seconds.
5. Power down and then re-start all devices.
6. On your PC, Download SiLa2 Universal Client for [Windows](https://gitlab.com/SiLA2/universal-sila-client/sila_universal_client/-/jobs/artifacts/master/download?job=build_and_test) or for [other OS](https://gitlab.com/SiLA2/universal-sila-client/sila_universal_client/-/jobs/artifacts/master/download?job=build_and_test).
7. Make sure [Java 8](https://adoptium.net/?variant=openjdk11) is installed,
8. On Windows, run the executable `/artifacts/back/target/usc.exe`. For other OS, use `java -jar back-0.10.0-SNAPSHOT-app.jar` in folder `/artifacts/back/target/`
9. Browse to [http://localhost:8080/](http://localhost:8080/) and Discover Devices. HiPeristaltic should be discovered now.

Continue with example calibration scenario:

10. Fill in the tubings (e.g., with pure water) by running a pump channel, for instance by using `StartPumpCalibration` with `PumpIndex : 0`, `RPM : 40`, `TargetRevolutions : 50` and with the correct direction set by `cw` for clockwise and `ccw` for counter-clockwise.
11. Once this task is finished, place a container which is already weighted when empty, and run `StartPumpCalibration` with desired parameters, note the `TargetRevolutions` value.
12. Weight the container again and calculate the pumped liquid weight. Convert the weight to volume in microliters using the density of the liquid.
13. Divide the volume with `TargetRevolutions` to obtain calibration factor with units "microliters per revolution". Use `SetPumpCalibration` function with this calibration factor.

# Advanced Installation

This sections outlines all steps used to create the installable images.

## Step 1: Rasberry Pi Setup

1. Insert the SD card into your PC.
2. Download Raspberry Pi Imager from [here](https://www.raspberrypi.com/software/) and run the software.
3. Select the Raspberry Pi device (Raspberry Pi 4 Model B was used for the publication).
4. For Operating System, select **Raspberry Pi OS (other)**.
5. Select the latest **Raspberry Pi OS Lite (64-bit)**.
6. Select the inserted SD card.
7. Press **Next**.
8. In **Edit Settings**
    - Under the **GENERAL** tab:
        - Check **Set hostname** and set to `hiernpi.local`.
        - Check **Set username and password** (example, please change) and set Username: `hte`, Password: `hte2025`.
    - Under the **SERVICES** tab:
        - Enable **SSH**.
        - Select "Use password authentication".
    - Under the **OPTIONS** tab:
        - Check **Eject media when finished**.
        - Uncheck **Enable telemetry**.
9. Press **Save**.
10. Press **Yes** to apply OS customization settings.
11. Press **Yes** to continue.
12. Once finished, remove the SD card and insert it into the Raspberry Pi.

## Step 2: Install SHH (required only for Windows PCs)

Install/enable OpenSSH on Windows using the following Administrative PowerShell command:
    1. Press **Windows + X** and open **Windows PowerShell (Admin)**.
    2. Run the following command:
    ```bash
    Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0

## Step 3: Setting Up Server Side

Following steps are tested with Rasberry Pi. For other OS, remove the following part from the 

### Method 1: Use the installation script

1. Download:
   
	```bash
	cd ~
	curl -L -O "https://github.com/gunakkoc/HiPeristaltic/raw/refs/heads/main/setup_hiperistaltic.sh"
	```
	
	The script is designed for Raspberry Pi OS. Remove the following lines for other Debian based distros.
	
	```bash
	# Enable UART
	echo "Enabling UART..."
	sudo raspi-config nonint do_serial_hw 0
	sudo raspi-config nonint do_serial_cons 1
	```
 
 2. Give execution permission:

	```bash
	sudo chmod +x ~/setup_hiperistaltic.sh
	```
		
3. Run the installation script:
   
	```bash
	sudo ./setup_hiperistaltic.sh
	```

### Method 2: Manual/custom installation

The typical installation steps are outlined here for Rasberry Pi. 

#### Setup Communication

-  Enable UART (recommended)
   ```bash
   sudo raspi-config nonint do_serial_hw 0
   sudo raspi-config nonint do_serial_cons 1
   ```

-  Alternatively, connect the MCU board (i.e., BIGTREETECH SKR MINI E3 V3.0) to the Rasberry Pi via USB. A new device should show up with the command `ls /dev/tty*` such as `/dev/ttyACM0`. Note this device name.

#### Install Mini-Forge and Dependencies:

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

#### Download HiPeristaltic:

1. Download **HiPeristaltic SiLa2/Python Interface**:
    ```bash
    curl -L -O "https://github.com/gunakkoc/HiPeristaltic/raw/refs/heads/main/silav2/HiPeristaltic.zip"
    ```

2. Unzip to the home directory:
    ```bash
    unzip HiPeristaltic.zip -d ~
    ```

#### Set up the startup script:

This script can be used to manually start the HiPeristaltic SiLa2 server.

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

#### Add systemd Service:

This to start HiPeristaltic SiLav2 server automatically when the system boots.

1. Create a systemd service file:
    ```bash
    sudo nano /etc/systemd/system/hiperistaltic.service
    ```

2. Type the following inside the file:
    ```bash
    [Unit]
    Description=Run HiPeristaltic SiLa2 Server
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


# Hardware Options

## Microcontroller platforms

To cover the most common microcontroller platforms, 3 firmwares are provided:
1. For STM32 platform: tested with STM32G0B1RET6.
	- Includes an utilizes a driver for TMC2209 stepper motor driver to maximize performance.
	- Requiries STM32Cube and STM32 debugger.
	- Both UART and USB are supported. Defaults to UART, switches to USB connection automatically if detected and until reboot.
2. For Arduino platform: tested with Arduino Uno Rev3(ATmega328P) and Arduino Mega 2560 (ATmega2560)
	- Can be compiled with other Arduino compatible platforms such as `stm32duino`
	- Can not be directly connected to Rasberry Pi models over UART. 
3. Raspberry Pi Pico, tested with Raspberry Pi Pico (RP2040) and Raspberry Pi Pico 2 (RP2350)
	- Can be connected Rasberry Pi models over UART. 
	
The GPIO pins needs to be adjusted for the specific case.

When connecting via USB, the condiguration file `HiPeristaltic.toml` needs to be adjusted, for instance:
	```
	"serial_port" =  "tty/USB0"
	```
	
## Stepper Motor Drivers

Basically all 2-phase stepper drivers that can operate with an step pin can be used, including:

1. A4988
2. DRV8825
3. TMC2208
4. TMC2209
5. TB6600
6. DM556

The recommended board that is BIGTREETECH SKR Mini E3 v3.0 comes with TMC2209 drivers embedded. Perhaps the cheapest alternative is using an Arduino+CNC Shield combo with A4988 drivers attached.

With STM32 firmware, one can setup U(S)ART as necessary and the included TMC2209 driver can be used for full feature access.

In any case, the active microstepping value (e.g., 1, 2, 4, 8) needs to be specified in the `HiPeristaltic.toml` file, per driver.

## Connectivity

The following connecitity options are available, depending on the selected MCU.

1. Networking (i.e., via Ethernet or WiFi) through SiLa2.
2. Serial over USB for direct PC to MCU communication, without SiLa2 layer.
3. Serial over UART, typically supported only by SBCs such as Rasberry Pi, used for direct communication as in USB.

# About Pump Configuration and Usage

The default configuration file is "HiPeristaltic.toml", and by default within the same folder as "HiPeristalticInterface.py". Usually, no modifications to this file are necessary. 

The calibration factor can be accessed under ```calibration_uL_per_Rev``` for each pump. Note that the provided SiLa2 client can be used to remotely change this parameter which is then immediately saved to the configuration file.

The Python interface can be used to control the pump without SiLa2. It is built with minimum dependencies, with only additional libraries being `numpy` and `serial`.
