# Jetson Orin Nano Super - Environment Issues and Solutions

This document records common environment issues and solutions encountered when working with **Jetson Orin Nano Super**.

---

## 📉 Browser Not Opening

**Issue**: The default `snapd` version on Jetson Orin Nano may prevent browsers like Firefox (installed via Snap) from launching properly.

### Solution (Install specific version of `snapd`)  
Source: [NVIDIA Developer Forum](https://forums.developer.nvidia.com/t/jetson-orin-nano-browser-issue/338580/6)

```bash
snap download snapd --revision=24724
sudo snap ack snapd_24724.assert
sudo snap install snapd_24724.snap
```

### If the issue comes back after some time:

```bash
sudo snap revert snapd
```

---

## Joystick Has No /dev/input/js0 Port

**Issue**: JetPack 6 does not include the `xpad` driver by default, so Xbox controllers won’t create the `js0` device.

### Solution (Compile and install `xpad` driver manually)
Source: [GitHub - jetpack6-joy](https://github.com/woawo1213/jetpack6-joy)

```bash
sudo apt update
sudo apt upgrade

git clone https://github.com/paroj/xpad.git
cd xpad
make
```
### Install the driver to the correct path
```bash
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/input/joystick
sudo cp xpad.ko /lib/modules/$(uname -r)/kernel/drivers/input/joystick
```

### Load the module
```bash
sudo depmod -a
sudo modprobe xpad
```

### Check if the driver was loaded successfully
```bash
sudo dmesg | grep xpad
```

### Check if the device exists
Note: your devices might be js1, js2, ...
```bash
ls -l /dev/input/js0
jstest /dev/input/js0
```
---
## Installing `Visual Studio Code`

**Issue**: Jetson Orin Nano does not come with VS Code by default, and newer versions are not compatible with ARM on older Ubuntu versions.

### Solution (Install older compatible version)
Source: [JetsonHacksNano/installVSCode](https://github.com/JetsonHacksNano/installVSCode)

```bash
# Set the version (1.85.2 is the latest compatible with Ubuntu 18.04)
VERSION=1.85.2

# Download and install
wget -N -O vscode-linux-deb.arm64.deb https://update.code.visualstudio.com/$VERSION/linux-deb-arm64/stable
sudo apt install ./vscode-linux-deb.arm64.deb
```