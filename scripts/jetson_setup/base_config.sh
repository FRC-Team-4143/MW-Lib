#!/bin/bash

# verify that the non-root user is set
if [ -z "$SUDO_USER" ]; then
    echo "ERROR: This script must be run with sudo from a non-root user"
    exit 1
fi

if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script can only be run as root"
    exit 1
fi

# Get IP address from command line argument, otherwise use default
if [ $# -ge 1 ]; then
    TEAM_NUMBER=$1
else
    TEAM_NUMBER=4143
fi

# Validate the team number is a number between 0 and 9999
if ! [[ "$TEAM_NUMBER" =~ ^[0-9]+$ ]]; then
    echo "ERROR: Team number must be a number"
    exit 1
fi

if [ "$TEAM_NUMBER" -lt 0 ] || [ "$TEAM_NUMBER" -gt 9999 ]; then
    echo "ERROR: Team number must be between 0 and 9999"
    exit 1
fi

# check if we also have a final octet provided, if so use that instead of 200
if [ $# -ge 2 ]; then
    FINAL_OCTET=$2
    if ! [[ "$FINAL_OCTET" =~ ^[0-9]+$ ]]; then
        echo "ERROR: Final octet must be a number"
        exit 1
    fi
    if [ "$FINAL_OCTET" -lt 1 ] || [ "$FINAL_OCTET" -gt 254 ]; then
        echo "ERROR: Final octet must be between 1 and 254"
        exit 1
    fi
else
    FINAL_OCTET=200
fi

# convert team number to ip address by splitting into octets and joining with dots
TEAM_STR=$(printf "%04d" $TEAM_NUMBER)
DEVICE_IP="10.${TEAM_STR:0:2}.${TEAM_STR:2:2}.${FINAL_OCTET}"

# figure out the subnet from the IP address
IFS='.' read -r -a ip_array <<< "$DEVICE_IP"
SUBNET="${ip_array[0]}.${ip_array[1]}.${ip_array[2]}"

# Detect the ethernet interface name (e.g. eno1, enp8p1s0, eth0)
ETH_INTERFACE=$(ip -o link show | awk '$2 ~ /^e/ {gsub(/[@:].*/, "", $2); print $2; exit}')
if [ -z "$ETH_INTERFACE" ]; then
    echo "ERROR: Could not detect an ethernet interface"
    exit 1
fi
echo "------------------------------------------------------------------"
echo "Ready to configure the device with the following settings:"
echo "Team number: $TEAM_NUMBER"
echo "Device IP address: $DEVICE_IP"
echo "Ethernet interface: $ETH_INTERFACE"
echo "Subnet: ${SUBNET}.0/24"
echo "------------------------------------------------------------------"
echo ""

read -p "Press enter to continue with the configuration, or Ctrl+C to cancel"


# write the team number to /etc/app/team so that it can be accessed by the app container
echo "Writing team number to /etc/app/team"
mkdir -p /etc/app
echo "$TEAM_NUMBER" > /etc/app/team

echo "Disabling desktop"
systemctl set-default multi-user.target

echo "Configuring static IP address for ethernet"

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

echo "Configuring fan to run at full speed on boot"

cat > /etc/systemd/system/fan-full-speed.service <<EOF
[Unit]
Description=Setup Jetson Runtime settings
After=network.target

[Service]
ExecStartPre=/bin/sleep 5
ExecStart=/bin/sh -c 'echo 64 > /sys/module/usbcore/parameters/usbfs_memory_mb'
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