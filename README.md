# Frogbot STM32

This repository contains the STM32 lower controller code for Frogbot.

## How to back up the existing code in the STM32

In case there is no backup of the build files currently running the board, do this:

1. Turn the robot OFF. Make sure the STM32 is completely turned off.
> [!IMPORTANT] For the Frogbot, your computer will power up the STM32. Make sure you turn the big battery OFF, or you may burn the STM32. For other robots, check whether the 3.3V cable is plugged.
2. Plug the ST-LINK debugger (the USB thingy) into your PC. Make sure the cables are properly connected to the STM32.
3. Launch `STM32CubeProgrammer` and click the green `Connect` button in the top-right corner. The logs should then show the device info. (If you do not have it, download it here for free: https://www.st.com/en/development-tools/stm32cubeprog.html)
4. In the address bar, ensure Address is 0x08000000 and Size matches your chip's Flash size (0x80000 for 512 KB, 0x100000 for 1 MB). For Frogboard, the Size is `0x100000`.
5. Click the arrow next to the `Read` tab and select `Save As...`, and make sure to set the filetype to `.hex`.
6. For Frogboard: To verify if the file is ok, open it in Notepad and read the first line. It MUST read exactly as `:020000040800F2`

***

## How to flash the STM32

This repository can be compiled directly into a hex file to flash the STM32. Start working from the root of this repository. Linux (wsl OK).

### 1. Build the Firmware in Terminal

Ensure `make` and the GCC ARM cross-compiler are installed and in your `PATH`:
```bash
arm-none-eabi-gcc --version
make --version
```

If missing, install: 
```bash
sudo apt install -y make gcc-arm-none-eabi binutils-arm-none-eabi
```

Then, from the project root containing the `Makefile`, run:
```bash
make -j$(nproc)
```

Let it run for a bit, and verify the build completes without errors and generates the binaries in `build/`.

3. Confirm output files:
```bash
ls -lh build/*.hex build/*.elf
```

There should be a `.hex`, `.elf`, `.bin` files inside `build/`. Here we use the `.hex` file.

You can freely copy, move, or rename the resulting `.hex` file. I strongly encourage you to save all builds in a backed-up folder.


### 2. Flash with STM32CubeProgrammer

If you do not have it, download it here for free: https://www.st.com/en/development-tools/stm32cubeprog.html

1. Turn the robot OFF. Make sure the STM32 is completely turned off.

> [!IMPORTANT] For the Frogbot, your computer will power up the STM32. Make sure you turn the big battery OFF, or you may burn the STM32. For other robots, check whether the 3.3V cable is plugged.

2. Plug the ST-LINK debugger (the USB thingy) into your PC. Make sure the cables are properly connected to the STM32.
3. Launch `STM32CubeProgrammer` and click the green `Connect` button in the top-right corner. The logs should then show the device info.
4. Select the `Erasing & Programming` icon (downward arrow pointing to a chip on the left navigation bar).
5. Next to "File Path", click `Browse` and select the `.hex` file from `build/` (the one just generated). After selecting, the start address field should read `0x08000000` for a hex file.
6. Configure Flashing Options:
- Check `Verify programming`
- Check `Run after programming`
7. Click `Start Programming`. Wait until it shows "File download complete, Verification...OK", then the board is flashed.