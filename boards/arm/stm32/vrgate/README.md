# VRGATE

Port a working Nuttx onto VRGATE, a STM32f405 based board.

## HOWTO

Requires corresponding apps from my repo.

```
./tools/configure.sh vrgate:usbnsh
```

Then follow the official Nuttx workflow.

## About ROMFS

Run the script `mkromfs.sh` within this directory and romfs will be generated.

`etcfs` contains init.d script for autostart. `romfs` currently contains `edid.bin` for convenience.

