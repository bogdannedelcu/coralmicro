#!/usr/bin/env bash
# grant_jtag_access.sh - run ONCE with sudo to let Claude drive the J-Link
# autonomously afterward (no sudo, non-interactive).
#
#   sudo bash /home/bogdan/work/coralmicro/grant_jtag_access.sh
#
# What blocks non-interactive JLinkExe is NOT permissions (the udev rule is
# already MODE=666 and the user is in plugdev) - it is the one-time J-Link
# firmware-update prompt on first use of this JLink software version. This
# script accepts that update non-interactively so subsequent runs by Claude
# (as the normal user) connect cleanly.
set -u
USERNAME="${SUDO_USER:-bogdan}"
echo "=== grant_jtag_access (user=$USERNAME) ==="

echo "--- 1. groups (plugdev for USB, dialout for /dev/ttyACM0) ---"
usermod -aG plugdev "$USERNAME" 2>/dev/null && echo "  + plugdev ok"
usermod -aG dialout "$USERNAME" 2>/dev/null && echo "  + dialout ok"

echo "--- 2. udev rule for J-Link (1366:*) world-accessible + reload ---"
RULE=/etc/udev/rules.d/99-jlink.rules
if [ ! -f "$RULE" ]; then
  cat > "$RULE" <<'EOF'
ATTR{idProduct}=="0101", ATTR{idVendor}=="1366", MODE="666"
ATTR{idProduct}=="0105", ATTR{idVendor}=="1366", MODE="666"
EOF
  echo "  installed $RULE"
else
  echo "  $RULE already present (MODE=666)"
fi
udevadm control --reload-rules && udevadm trigger && echo "  udev reloaded"

echo "--- 3. prime J-Link firmware (accept update non-interactively) ---"
# Feed the firmware-update prompt a stream of newlines/yes so it auto-accepts,
# then connect+halt+go+quit. Run as the normal user so the resulting J-Link
# state matches how Claude will invoke it.
PRIME_CMD=$(mktemp)
printf 'halt\ngo\nqc\n' > "$PRIME_CMD"
echo "  connecting (this updates J-Link firmware if needed)..."
yes '' | timeout 90 sudo -u "$USERNAME" JLinkExe \
    -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -autoconnect 1 -NoGui 1 \
    -CommandFile "$PRIME_CMD" 2>&1 | grep -iE \
    "Updating|firmware|Cortex-M7|Connecting|Connected|Reset|Halt|FAILED|VTref|J-Link>" \
    | head -25
rc=${PIPESTATUS[1]}
rm -f "$PRIME_CMD"
echo "  prime exit=$rc"

echo "--- 4. verify normal-user access (no sudo) ---"
VERIFY=$(mktemp)
printf 'halt\ngo\nqc\n' > "$VERIFY"
sudo -u "$USERNAME" timeout 40 JLinkExe -device MIMXRT1176xxxA_M7 -if SWD \
    -speed 4000 -autoconnect 1 -NoGui 1 -CommandFile "$VERIFY" 2>&1 \
    | grep -iE "Cortex-M7|Connected|Halt|FAILED|Identified" | head -8
rc2=${PIPESTATUS[1]}
rm -f "$VERIFY"
echo "  verify exit=$rc2"

echo ""
if [ "${rc2:-1}" = "0" ]; then
  echo "=== OK: Claude can now drive JLinkExe as $USERNAME (no sudo). ==="
else
  echo "=== If verify failed: re-plug the J-Link USB once, then tell Claude. ==="
fi
