#!/usr/bin/env bash
echo 'KERNEL=="ttyTHS*", MODE="0666"' | sudo tee /etc/udev/rules.d/92-hoverboard.rules
sudo udevadm control --reload-rules
sudo udevadm trigger