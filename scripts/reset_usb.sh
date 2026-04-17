#!/bin/bash
# Reset XHCI USB host controller to recover dead USB port
# Run with: sudo bash scripts/reset_usb.sh
set -e
CTRL="0000:04:00.4"
echo "Unbinding XHCI controller $CTRL..."
echo "$CTRL" > /sys/bus/pci/drivers/xhci_hcd/unbind
sleep 2
echo "Rebinding XHCI controller $CTRL..."
echo "$CTRL" > /sys/bus/pci/drivers/xhci_hcd/bind
sleep 2
echo "Done. USB bus 3 should be back."
lsusb | grep -E "1fc9|18d1" || echo "No NXP/Google device found yet - plug in the board"
