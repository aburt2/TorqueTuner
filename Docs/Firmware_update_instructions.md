# Torquetuner (firmware version: 20221020)

- [Torquetuner (Firmware version: 20221020)](#torquetuner-firmware-version-20221020)
  - [First time firmware upload instructions](#first-time-firmware-upload-instructions)
    - [Flash Moteus Board](#flash-moteus-board)
      - [Recompiling the Moteus firmware](#recompiling-the-moteus-firmware)
      - [Setup flash with debug USB stick:](#setup-flash-with-debug-usb-stick)
    - [Flash TinyPico Board](#flash-tinypico-board)
      - [Install PlatformIO](#install-platformio)
      - [Clone the Torquetuner repository](#clone-the-torquetuner-repository)
      - [Open software project and flash it to the Torquetuner](#open-software-project-and-flash-it-to-the-torquetuner)
      - [Test Torquetuner](#test-torquetuner)
  - [Other Documentation](#other-documentation)
  - [Firmware information](#firmware-information)

## First time firmware upload instructions
[Modified from the T-Stick firmware update instructions](https://github.com/aburt2/T-Stick/blob/master/Docs/Firmware_update_instructions.md)
_INSTALL ALL DEPENDENCIES AND REAL ALL OBSERVATIONS BEFORE UPLOAD !_
### Flash Moteus Board

#### Recompiling the Moteus firmware:
1. git clone openocd: https://github.com/mjbots/openocd.git
2. In the cloned reporisitory, run:
```
sudo apt install autotools-dev automake autogen autoconf libtool libusb-1.0-0-dev
./bootstrap
./configure --prefix=/usr
make 
sudo checkinstall --pkgname=openocd --pkgversion=0.10.1 --pkgrelease=0 --requires="libusb-1.0-0-dev" --exclude="/home" -y 
```

You might get errors from running `make`. It might be because of the missing submodules, which can be resolved with `git submodule update --recursive` and then returning to step 1.
Or it can also be solved with `./configure --prefix=/usr --disable-werror`.

3. Run the following set of instructions:
```
sudo apt install binutils-arm-none-eabi wget
sudo usermod -aG plugdev $USER
wget https://raw.githubusercontent.com/mjbots/openocd/master/contrib/60-openocd.rules
sudo cp 60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. Logout and log in back again to make the aforementioned settings effective

5. Under `Documents`, clone moteus: https://github.com/IDMIL/Moteus/tree/RotaRep branch.
6. In the `moteus` directory, build the moteus firmware:
```
tools/bazel build --config=target //:target --verbose_failures
```
potential files to modify if errors occur during build:
- /home/idmil/.cache/bazel/_bazel_idmil/7a1dab6e225502b7cb7385a0c98596b0/external/com_github_mjbots_bazel_toolchain/toolchain/tools/llvm_release_name.py
- /home/idmil/.cache/bazel/_bazel_idmil/e1e3d4966c1ea71e0002f6cdeaa52362/external/com_grail_bazel_toolchain/toolchain/tools/llvm_release_name.py

7. (optional) test `tools/bazel test --config=target //:target`

#### Setup flash with debug USB stick:

8. Connect the USB stick to the board using the JST ZH-6. Then connect the whole system as per the [Hardware Assemblage section](#steps-for-assembling-and-testing-the-14-poles-motor-moteus-r45-dev-kit). And plug in the USB stick into the computer.

9. Flash with debug USB stick: `./fw/flash.py`

Upon succesful flash, the following lines will be output:
```
** Programming Finished **
** Verify Started **
** Verified OK **
** Resetting Target **
...
shutdown command invoked
```

Note: if you encounter the error message:
```
Info : STLINK V2J29S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.223809
Error: init mode failed (unable to connect to the target)
```
check the different connections related to the power supply.

10. Launch Moteus GUI: `python3 -m moteus_gui.tview --devices=1`
11. In the Moteus GUI, check the I2C state and status:
```
d i2c.state
d i2c.status
```
For now state should be READY or LISTEN and status ERROR or BUSY (with no other i2c device connected).

### Flash TinyPico Board

#### Install PlatformIO

To download and install PlatformIO, follow the instructions at [https://platformio.org/platformio-ide](https://platformio.org/platformio-ide).

We recomment using PlatformIO under Visual Studio Code, but you can also coose another editor.

#### Clone the Torquetuner repository

Clone this repository using `git clone https://github.com/IDMIL/TorqueTuner.git`. Alternatively, you can download the repository as a zip file at [https://github.com/IDMIL/TorqueTuner](https://github.com/IDMIL/TorqueTunerk). Take note of the folder location.

#### Open software project and flash it to the Torquetuner

- Open the Torquetuner firmware project (folder **software** in the Torquetuner repository folder) in VSC/PlatformIO. You can get help on how to use PlatformIO at [https://docs.platformio.org/en/latest/core/quickstart.html](https://docs.platformio.org/en/latest/core/quickstart.html)
- You can make any necessary changes on the firmware before flashing (e.g., changing Torquetuner ID)
- If it is the first time flashing, you may see an error pointing to the ESP32 inet.h file. The file requires manual fixing. Check the issue at [https://github.com/mathiasbredholt/libmapper-arduino/issues/3](https://github.com/mathiasbredholt/libmapper-arduino/issues/3)

When ready, you need to flash both the firmware and the filesystem image. Choose the proper platform accordingly (*tinypico*) and use the PlatformIO menu to flash the image to the Torquetuner.

#### Test Torquetuner

After flashing, you can use the VSC/PlatformIO serial monitor to check if the Torquetuner module is booting properly. You should see Torquetuner booting process.

You can also interact with the controller using the following commands:

- 'reboot' to reboot

## Other Documentation

[Torquetuner connection guide – v1.0](./connection_guide_v1.md)

[How to build a Torquetuner Module](./Build_guide_v1.md)

## Firmware information

Torquetuner V1 - TinyPico - USB - WiFi
Input Devices and Music Interaction Laboratory (IDMIL)  
