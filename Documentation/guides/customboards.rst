=============================================
Custom Boards and Custom Apps Location How-To
=============================================

As explained in :doc:`../quickstart/configuring`, supported boards (also known
as "in-tree" boards) are configured using a standard syntax:

    .. code-block:: console

      $ cd nuttx
      $ ./tools/configure.sh -l board-name:config-name
        Copy files
        Select CONFIG_HOST_LINUX=y
        Refreshing...

Sometimes it is not appropriate, or not wanted, to add a new or custom board to
the Nuttx boards tree itself. If so, the board can be defined out-of-tree in a
custom folder and still easily built.

Similarly, an application can be located in a custom folder rather than within
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

.. Note::
   If you subsequently run a ``make distclean`` operation, then those settings will be lost.

Custom Apps
===========

Many people use the generic apps/ directory and add custom apps there.

If there is a wish to keep applications out-of-tree, then this section explains how to 

* Add a new folder to the in-tree Apps folder for your own apps
* Create a custom application directory from scratch, using ``hello.c`` as an example.

Creating a Distinct Custom Application Directory
------------------------------------------------

The custom Application folder (``CustomApps`` in this example)need only contain the minimum three files:
* ``Makefile``
* ``Kconfig``
* ``hello.c``


Makefile
^^^^^^^^

The custom application directory must include a Makefile to nake all of the make targets expected by
the NuttX build and must generate an archive called libapps.a in the top-level of the custom directory
structure. 

The Makefile has just those minimum required targets:

    .. code-block:: console

      APPDIR = ${shell pwd}
 
      -include $(TOPDIR)/Make.defs
 
      # files
 
      CSRCS = hello.c
      COBJS = hello.o
 
      ROOTDEPPATH = --dep-path .
 
      # Build targets
 
      all: libapps.a
      .PHONY: dirlinks context preconfig depend clean clean_context distclean
      .PRECIOUS: libapps$(LIBEXT)
 
      # Compile C Files
 
      $(COBJS): %$(OBJEXT): %.c
      $(call COMPILE, $<, $@)
 
      # Add object files to the apps archive
 
      libapps.a: $(COBJS)
        $(call ARCHIVE, libapps.a, $(COBJS))
 
      # Create directory links
 
      dirlinks:
 
      # Setup any special pre-build context
 
      context:
 
      # Setup any special pre-configuration context
 
      preconfig:
 
      # Make the dependency file, Make.deps
 
      depend: Makefile $(CSRCS)
        $(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep
 
      # Clean the results of the last build
 
      clean:
        $(call CLEAN)
 
      # Remove the build context and directory links
 
      clean_context:
 
      # Restore the directory to its original state
 
      distclean: clean clean_context
        $(call DELFILE, Make.dep)
 
      # Include dependencies
     
      -include Make.dep
  
Kconfig
^^^^^^^

A Kconfig file must be included but need not be populated with any meaningful options.
This is a place where you can add settings to generate customized builds of your custom
application and/or choose which of your apps to include.

In the minimum case, Kconfig is only:

    .. code-block:: console

      # For a description of the syntax of this configuration file,
      # see the file kconfig-language.txt in the NuttX tools repository.
      #

or

    .. code-block:: console

      # For a description of the syntax of this configuration file,
      # see the file kconfig-language.txt in the NuttX tools repository.
      #



hello.c
^^^^^^^

The custom application must actually compile some source files in order to generate the required
libapps.a archive. One of these source files must include the ``main()`` entry point to the
application.

The function of this main() entry point simply to bring-up the full application. It is called
at the completion of OS initialization.

What this application initialization entry point does, how it interacts with the rest of your
application, and where the rest of you application code is located is of no concern to the OS.

Only this one entry point is needed.

For this "Hello, World!" application ``custom_main()`` is the application entry point:

    .. code-block:: console

      #include <stdio.h>
 
      int custom_main(int argc, char *argv[])
      {
        printf("Hello, World!!\n");
        return 0;
      }

Building with the Custom Application Directory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to build with the new custom configuration, you will need the following in your configuration:

:menuselection:`CONFIG_APPS_DIR="../CustomApps"`

:menuselection:`CONFIG_USER_ENTRYPOINT="custom_main"`

Note that you can only access the ``../CustomApps/Kconfig`` configuration file if ``CONFIG_APPS_DIR`` is set
to ``../CustomApps`` BEFORE ``make menuconfig`` is executed

This can be done by

* hand-editing the .config file before running make menuconfig, which is rarely a good idea
* Using ``kconfig-tweak --set-str CONFIG_APPS_DIR ../CustomApps``
* select the CustomApps directory as a command line option at the time the board is configured:
  
      .. code-block:: console  

        ./tools/configure.sh -a ../CustomApps <board>:<config>

  or

      .. code-block:: console  

        .tools/configure.sh -l ../CustomBoards/MyCustomBoardName/MyCustomConfig

Then just build as you normally would. When you execute the program with built with the custom application directory you should see:
Hello, World!!


