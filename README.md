# AudioMoth-Edge Impulse
A modified version of the [AudioMoth standard firmware](https://github.com/OpenAcousticDevices/AudioMoth-Project). This software performs live classification of samples using a neural network from Edge Impulse and saves audio clips containing a specific sound.

For more details, visit [AudioMoth Labs](https://www.openacousticdevices.info/labs).

### Usage ###

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace the ```src/main.c``` from AudioMoth-Project with the ```src/main.c``` from this repository. Put all the remaining ```src/*.c``` files and all the ```src/*.h``` files from this repository into the ```/src/``` and ```/inc/``` folders of the AudioMoth-Project repository. Copy the ```edgeimpulse/``` and ```dsplib/``` folder (including its contents and subfolders) from this repository into the AudioMoth-Project repository. Replace the `build/Makefile` from AudioMoth-Project with the `build/Makefile` from this repository. 

In `fatfs/inc/ffconf.h` make the following changes

- Disable support for long file names: ```#define FF_USE_LFN		0```
- Disable support for exFAT filesystem: ```#define FF_FS_EXFAT		0```
- Enable tiny buffer configuration: ```#define FF_FS_TINY		1```

The SD card used with AudioMoth Edge Impulse needs to have a `CONFIG.TXT`. An example CONFIG.TXT file is included in this repo and can be customized. 

#### !! NOTE ####
**Currently AudioMoth with Edge Impulse only supports FAT32-formatted SD cards**

## Steps to Build Firmware with different Edge Impulse models ##

1. Train a different neural network (using MFCC for audio) on the [Edge Impulse](https://edgeimpulse.com/) platform. Currently AudioMoth only supports 16kHz or 8kHz with 1 second audio samples. Go to the "Deployment" tab, build the C++ library, and download the ZIP file (and unzip it). 
2. In the `firmware/Makefile` set the `EI_MODELPATH =` variable to the path of the Edge Impulse ZIP Model folder.
3. In `firmware/src/main.c` set the `#define EI_MODEL_FREQUENCY` to `KHZ_16` for a 16 kHz Edge Impulse model or `KHZ_8` for a 8 kHz Edge Impulse model. The AudioMoth will record audio files at the same frequency as the Edge Impulse model.
4. Run `make clean` and `make` to build the firmware with the new model.

## Troubleshooting ## 

See [AudioMoth LED Guide](https://www.openacousticdevices.info/led-guide) for the meaning of the LED lights and troubleshooting. 

#### 3 Quick Flashes of Red & Green LED ####

1. Check that the Edge Impulse model sample rate (8 kHz or 16 kHz) matches the AudioMoth sample rate setting (`firmware/src/main.c` set the `#define EI_MODEL_FREQUENCY`)
2. If this problem still persists, the Edge Impulse model uses too much RAM (likely during the MFCC block). Reduce the parameters used in MFCC and try the updated model. Models with RAM usage > 20 kB are likely not to work.

### License ###

Copyright 2023 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
