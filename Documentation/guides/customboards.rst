====================
Custom Boards How-To
====================

As explained in :doc:'../quickstart/configuring', supported boards (also known
as "in-tree" boards) are configured using a standard syntax:

    .. code-block:: console

      $ cd nuttx
      $ ./tools/configure.sh -l board-name:config-name
        Copy files
        Select CONFIG_HOST_LINUX=y
        Refreshing...

Sometimes it is not appropriate, or wanted, to add a new or custom board to the
Nuttx boards tree itself. If so, the board can be defined out-of-tree in a
custom folder and still built easily.

Similarly, an application can be located in a custom folder, rather than within
the nuttx-apps directory tree.

Custom Boards
=============

The same set of files as provided for in tree boards is required (i.e. configs,
Kconfig, scripts, etc.) but these can be placed in a directory of your choice.

In this example, the files are assumed to exist in:
 ``../nuttx/CustomBoards/MyCustomBoardName`` 

    .. code-block:: console

      $pwd
      /home/nuttx/nuttx
      $ ls -1 ../CustomBoards/MyCustomBoardName
      configs
      helpers
      include
      Kconfig
      scripts
      $ ls ../CustomBoards/MyCustomBoardName/configs
      nsh
      MyCustomConfig
      $


To build the custom board, the syntax is slightly different to in-tree boards and configs:

    .. code-block:: console

      $ .tools/configure -l ../CustomBoards/MyCustomBoardName/MyCustomConfig
      Copy files
      Select CONFIG_HOST_LINUX=y
      Refreshing...

Kconfig Settings
----------------
Once the board is configured, to ensure subsequent builds run correctly, There
are some Kconfig settings that need to be set. There are:

:menuselection:`Board Selection --> Custom Board Configuration --> Custom Board Name`

:menuselection:`Board Selection --> Custom Board Configuration --> Relative custom board directory`

and should be set to suit your board name and directory location.

.. Tip::
   If you subsequently run a ``make distclean`` operation, then those settings will be lost.

Custom Apps
===========

