README
======
######## WARNING:THIS LIBOPENCM3 FORK USES CMAKE AS BUILD SYSTEM ##########

This fork were taken because I had many files that the original libopencm3 git don't have. I will work on them in whenever I'll have time to verify the code written by me. The code committed here are provided "as is" and without any warranty. For more information see the terms of the GNU Lesser General Public License as published by the Free Software Foundation. As well as the code here complies with the commitment of being written from scratch based no the vendor documentation.
When I'll feel confident to make a pull request, I supposed to do so.

The original repo is in https://github.com/libopencm3/libopencm3

###########################################################################

The libopencm3 project aims to create an open-source firmware library for
various ARM Cortex-M microcontrollers.

Currently (at least partly) supported microcontrollers:

 - ST STM32 F0xx/F1xx/F2xx/F30x/F37x/F4xx/F7xx/H7xx series
 - ST STM32 G0xx G4xx L0xx L1xx L4xx series
 - Atmel SAM3A/3N/3S/3U/3X series, as well as SAMDxx and friends
 - NXP LPC1311/13/17/42/43
 - Stellaris LM3S series (discontinued, without replacement)
 - TI (Tiva) LM4F series (continuing as TM4F, pin and peripheral compatible)
 - EFM32 Gecko series (only core support)
 - Freescale Vybrid VF6xx
 - Qorvo (formerly ActiveSemi) PAC55XX
 - Synwit SWM050
 - Nordic NRF51x and NRF52x

The library is written completely from scratch based on the vendor datasheets,
programming manuals, and application notes. The code is meant to be used
with a GCC toolchain for ARM (arm-elf or arm-none-eabi), flashing of the
code to a microcontroller can be done using the OpenOCD ARM JTAG software.


Status and API
--------------

The libopencm3 project is (and presumably, always will be) a work in progress.
Not all subsystems of all microcontrollers are supported, yet, though some parts
have more complete support than others.

Prior to version 0.8.0, the api was largely in flux.  Attempts were made to provide
backwards compatibility, but this was not always considered critical.

From 0.8.0 to 1.0, we'll atempt to follow semver, but **EXPECT CHANGES**, as we
attempt to clear up old APIs and remove deprecated functions.  The 0.8.0 tag was
placed to provide the "old stable" point before all the new code started landing.

_preview_ code often lands in the "wildwest-N" branches that appear and disappear
in the repository.  Pull requests marked as "merged-dev" will be in this branch,
and will be closed when they merge to master.  This is useful for bigger
interdependent patch sets, and also allows review of merge conflicts in public.

From 1.0, expect to follow semver, with functions (and defines!) being deprecated for
a release before being removed.

_TIP_: Include this repository as a Git submodule in your project to make sure
     your users get the right version of the library to compile your project.
     For how that can be done refer to the
     [libopencm3-template](https://github.com/libopencm3/libopencm3-template) repository.

Prerequisites
-------------

Building requires Python (some code is generated).

**For Ubuntu/Fedora:**

 - An arm-none-eabi/arm-elf toolchain.

**For Windows:**

 Download and install:

 - msys - http://sourceforge.net/projects/mingw/files/MSYS/Base/msys-core/msys-1.0.11/MSYS-1.0.11.exe
 - Python - https://www.python.org/downloads/windows/ (any release)
 - arm-none-eabi/arm-elf toolchain (for example this one https://launchpad.net/gcc-arm-embedded)

Run msys shell and set the path without standard Windows paths (adjusting to your version of Python), so Windows programs such as 'find' won't interfere:

    export PATH="/c//Program Files/Python 3.9:/c/ARMToolchain/bin:/usr/local/bin:/usr/bin:/bin"

After that you can navigate to the folder where you've extracted libopencm3 and build it.

Toolchain
---------

The most heavily tested toolchain is "gcc-arm-embedded"
https://launchpad.net/gcc-arm-embedded

Other toolchains _should_ work, but they have not been nearly as well tested.
Toolchains targeting Linux, such as "gcc-arm-linux-gnu" or the like are
_not_ appropriate.

_NOTE_: We recommend that you use gcc-arm-embedded version 4.8 2014q3 or newer
to build all platforms covered by libopencm3 successfully.

Building
--------

    $ mkdir build
    $ cd build
    $ cmake .. -G'MSYS Makefiles' -DCMAKE_BUILD_TYPE=<Debug or Release> -DDEVICES=<devices to build the "libopencm3_<device>.a">

    $ make

Coding style and development guidelines
---------------------------------------

See HACKING.


License
-------

The libopencm3 code is released under the terms of the GNU Lesser General
Public License (LGPL), version 3 or later.

See COPYING.GPL3 and COPYING.LGPL3 for details.

Community
---------

 * Our [![Gitter channel](https://badges.gitter.im/libopencm3/discuss.svg)](https://gitter.im/libopencm3/discuss)
 * Our IRC channel on the libera.chat IRC network is called #libopencm3

Mailing lists
-------------

 * Developer mailing list (for patches and discussions):
   https://lists.sourceforge.net/lists/listinfo/libopencm3-devel

 * Commits mailing list (receives one mail per `git push`):
   https://lists.sourceforge.net/lists/listinfo/libopencm3-commits


Website
-------

 * http://libopencm3.org - contains daily autogenerated API documentation

