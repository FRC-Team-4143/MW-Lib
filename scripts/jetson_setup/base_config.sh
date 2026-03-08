#!/bin/bash

# Get IP address from command line argument, otherwise use default
if [ $# -ge 1 ]; then
    TEAM_NUMBER=$1
    echo "Using provided team number $TEAM_NUMBER"
else
    echo "Using default team number 4143"
    TEAM_NUMBER=4143
fi

if ! [[ "$TEAM_NUMBER" =~ ^[0-9]+$ ]]; then
    echo "Error: Team number must be a number"
    exit 1
fi

# check if we also have a final octet provided, if so use that instead of 200
if [ $# -ge 2 ]; then
    FINAL_OCTET=$2
    if ! [[ "$FINAL_OCTET" =~ ^[0-9]+$ ]]; then
        echo "Error: Final octet must be a number"
        exit 1
    fi
    if [ "$FINAL_OCTET" -lt 1 ] || [ "$FINAL_OCTET" -gt 254 ]; then
        echo "Error: Final octet must be between 1 and 254"
        exit 1
    fi
else
    FINAL_OCTET=200
fi

if [ "$TEAM_NUMBER" -lt 0 ] || [ "$TEAM_NUMBER" -gt 9999 ]; then
    echo "Error: Team number must be between 0 and 9999"
    exit 1
fi

# write the team number to /etc/app/team so that it can be accessed by the app container
mkdir -p /etc/app
echo "$TEAM_NUMBER" > /etc/app/team

# convert team number to ip address by splitting into octets and joining with dots
TEAM_STR=$(printf "%04d" $TEAM_NUMBER)
DEVICE_IP="10.${TEAM_STR:0:2}.${TEAM_STR:2:2}.${FINAL_OCTET}"
echo "Using device IP address $DEVICE_IP"

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

# Detect the ethernet interface name (e.g. eno1, enp8p1s0, eth0)
ETH_INTERFACE=$(ip -o link show | awk '$2 ~ /^e/ {gsub(/[@:].*/, "", $2); print $2; exit}')
if [ -z "$ETH_INTERFACE" ]; then
    echo "ERROR: Could not detect an ethernet interface"
    exit 1
fi
echo "Detected ethernet interface: $ETH_INTERFACE"

# Load in systemd-networkd scripts
cat > /etc/systemd/network/20-wired.network <<EOF
# Static IP configuration for ethernet port
# Sets the IP address for this computer
[Match]
Name=${ETH_INTERFACE}

[Link]
RequiredForOnline=yes

[Network]
DHCP=no
LinkLocalAddressing=no
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