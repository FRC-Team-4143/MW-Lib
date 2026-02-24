#!/bin/bash

# Get IP address from command line argument, otherwise use default
if [ $# -ge 1 ]; then
    DEVICE_IP=$1
    echo "Using provided IP address $DEVICE_IP"
else
    echo "Using default IP address 10.41.43.200"
    DEVICE_IP=10.41.43.200
fi

# figure out the subnet from the IP address
IFS='.' read -r -a ip_array <<< "$DEVICE_IP"
SUBNET="${ip_array[0]}.${ip_array[1]}.${ip_array[2]}"
echo "Using subnet $SUBNET.0/24 for network configuration"

if [ "$EUID" -ne 0 ]; then
    echo "This script can only be run as root"
    exit
fi

# verify that the non-root user is set
if [ -z "$SUDO_USER" ]; then
    echo "This script must be run with sudo from a non-root user"
    exit 1
fi

echo "Disabling desktop"
systemctl set-default multi-user.target

# Load in systemd-networkd scripts
cat > /etc/systemd/network/20-wired.network <<EOF
# Static IP configuration for ethernet port
# Sets the IP address for this computer
[Match]
Name=eno1

[Network]
Address=${DEVICE_IP}/24
Gateway=${SUBNET}.1
DNS=8.8.8.8 1.1.1.1
EOF

# Switch from NetworkManager to systemd-networkd on next reboot
systemctl disable NetworkManager
systemctl mask NetworkManager
systemctl unmask systemd-networkd
systemctl enable systemd-networkd

# add to dialout
usermod -aG dialout $SUDO_USER

echo "Configuring SD card automount"

# Configures SD automount
cat > /etc/systemd/system/mnt-sd.mount <<EOF
[Unit]
Description=Mount External MicroSD part1 at /mnt/sd

[Mount]
What=/dev/disk/by-path/platform-3400000.sdhci-part1
Where=/mnt/sd
Type=vfat
Options=noauto,nofail
EOF

cat > /etc/systemd/system/fan-full-speed.service <<EOF
[Unit]
Description=Set Fan speed to max

[Service]
ExecStart=/usr/bin/jetson_clocks --fan
Type=oneshot

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable fan-full-speed.service

cat > /etc/udev/rules.d/99-automount-sd.rules <<EOF
# Automounts SD Card on nvidia jetson orin
ACTION=="add",
SUBSYSTEM=="block",
ENV{DEVLINKS}=="*/dev/disk/by-path/platform-3400000.sdhci-part1*", 
ENV{SYSTEMD_WANTS}="mnt-sd.mount"
EOF

echo "Configuring docker"
cat > /etc/docker/daemon.json <<EOF
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "default-runtime": "nvidia",
    "insecure-registries" : ["192.168.1.0/24", "${SUBNET}.0/24"]
}
EOF
usermod -aG docker $SUDO_USER
systemctl restart docker

echo "Setup complete, please reboot to apply all changes"