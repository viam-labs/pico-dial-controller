#!/bin/sh
# Installs a udev rule granting non-root access to the Pico HID Encoder Controller.
# Runs automatically on first module install on the robot.

RULE='SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666"'
RULE_FILE=/etc/udev/rules.d/99-pico-hid.rules

if [ "$(uname)" != "Linux" ]; then
    exit 0
fi

echo "$RULE" > "$RULE_FILE"
udevadm control --reload-rules
udevadm trigger
