====================
Custom Boards How-To
====================
Sometimes it is not appropriate, or wanted, to include a new or custom board in
the Nuttx boards tree itself. If so, the board can be defined out-of-tree in a
custom folder and still built easily.

Similarly, an application can be located in a custom folder, rather than within
the nuttx-apps directory tree.

Custom Boards
=============

Standard supported boards are defined within the boards directory tree by
architecture, processor family then the actual board name. For example:

nuttx/boards/risc-v/esp32c/esp32c3-devkit

The specific configuration required is then detailed under a configs
subdirectory where a defconfig file resides. For example:

nuttx/boards/risc-v/esp32c/esp32c3-devkit/configs/nsh

When nuttx is configured for a supported board, the syntax used assumes that the
board and config files are located in the proscribed locations.

A custom board can be located anywhere, 

Custom Apps
===============
