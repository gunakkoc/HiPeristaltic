#!/bin/bash

# Enable UART
echo "Enabling UART..."
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1

# Update system
echo "Updating system..."
sudo apt update -y

# Install necessary dependencies
echo "Installing necessary dependencies..."
sudo apt install -y git curl unzip sudo apt-transport-https

# Download and install Miniforge
echo "Downloading and installing Miniforge..."
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh -b

# Initialize conda
echo "Initializing conda..."
~/miniforge3/bin/conda init
source ~/.bashrc

# Create conda environment with Python 3.9
echo "Creating conda environment 'hiperis' with Python 3.9..."
conda create -n hiperis python=3.9 -y
conda activate hiperis

# Ensure pip is installed
echo "Ensuring pip is installed..."
conda install pip -y

# Install necessary Python libraries
echo "Installing necessary Python libraries..."
pip install numpy pyserial toml sila2 sila2[codegen] sila2[cryptography]

# Install Git
echo "Installing Git..."
sudo apt install -y git

# Download HiPeristaltic
echo "Downloading HiPeristaltic..."
curl -L -O "https://github.com/HiPeristaltic/sila2/HiPeristaltic.zip"

# Unzip HiPeristaltic
echo "Unzipping HiPeristaltic..."
unzip HiPeristaltic.zip -d ~

# Create run_hiperistaltic.sh script
echo "Creating run_hiperistaltic.sh script..."
cat <<EOL > ~/run_hiperistaltic.sh
#!/bin/bash
# Get the first IPv4 address (excluding loopback)
CA_PATH=~/HiPeristaltic/HiPeristaltic_CA.pem
IP_ADDR=\$(hostname -I | awk '{print \$1}')
source ~/miniforge3/etc/profile.d/conda.sh
conda activate hiperis
cd ~/HiPeristaltic
python -m HiPeristaltic --ip-address "\$IP_ADDR" --port 50052 --ca-export-file "\$CA_PATH"
EOL

# Make the script executable
echo "Making run_hiperistaltic.sh executable..."
sudo chmod +x ~/run_hiperistaltic.sh

# Create systemd service
echo "Creating systemd service for HiPeristaltic..."
cat <<EOL | sudo tee /etc/systemd/system/hiperistaltic.service > /dev/null
[Unit]
Description=Run HiPeristaltic Sila2 Server
After=network.target

[Service]
WorkingDirectory=/home/hte/
ExecStart=/bin/bash /home/hte/run_hiperistaltic.sh
Restart=always

[Install]
WantedBy=multi-user.target
EOL

# Enable and start the service
echo "Enabling HiPeristaltic service to run on boot up."
sudo systemctl enable hiperistaltic

# Prompt for reboot
read -p "The setup is complete. Would you like to reboot now? (y/n): " answer

if [[ "$answer" == "y" || "$reboot_choice" == "Y" ]]; then
    echo "Rebooting the system..."
    sudo reboot
else
    echo "Reboot skipped. Please reboot the system manually to apply all changes."
fi
