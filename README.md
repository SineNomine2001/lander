# lander
Cambridge mars lander 2020-2021

Guide to starting this application from Cambridge:

Getting Started in Microsoft Windows
To work on the Mars Lander exercise, you need to install a C++ compiler on your PC.

First, install Microsoft's Visual Studio Community 2019 from here. The installer will ask you precisely what it is you wish to install. In the Workloads tab, select Desktop development with C++, add the most recent version of the optional Windows 10 SDK in the Installation details pane on the right (this may be ticked to install by default), then confirm that the NuGet package manager is selected from the Individual components tab. Press Install.
Installing Visual Studio after signing in to your Microsoft account should result in a permanently licensed installation on your PC. You can check this by launching Visual Studio, clicking on the Help menu, then About Microsoft Visual Studio, then License Status. If the license is for a 30 day trial period only, sign in to your Microsoft account using the link on the License Status page: this should automatically upgrade your license.
Here is a quick guide to getting started with the Mars Lander exercise:

Unzip the supplied Python and C++ source code into your Documents folder. This puts the source files in a new folder called lander.
Start Visual Studio. The first time you do this, you may be offered the opportunity to Start with a familiar environment by choosing between various Development Settings: select the default General option.
From the Get started dialog, select Create a new project (alternatively, and when running Visual Studio not for the first time, from the File menu select New and then Project). A pop-up dialog will appear. Select Empty Project (C++), type lander into the Project name box and press Create. From the Project menu, select Add Existing Item, navigate to the lander folder under Documents, select the three C++ source files lander.cpp, lander.h and lander_graphics.cpp (but not spring.cpp), and press Add.
From the Project menu, select Manage NuGet Packages. In the Browse tab search for nupengl, select the nupengl.core package and press Install. Click OK when the Preview Changes dialog pops up. This downloads and installs the various OpenGL graphics libraries required by the Mars Lander source code.
You can now edit the source code by double clicking on lander.cpp under Source Files in the Solution Explorer tab (if you cannot see this tab, you can enable it under the View menu).
Unlike Python, all C++ programs must first be compiled (i.e. translated into machine code) before they can be run.
The Build menu contains options for compiling the program, while the Debug menu is for running and debugging. Spend some time reading the Help pages for more information.
To compile and run the spring.cpp program for Assignment 3, repeat these instructions except: choose some other name for the project (not lander); add the spring.cpp source code file instead of the three lander source files; and skip the NuGet step for adding the OpenGL libraries.
If you experience any issues when trying to run your code (e.g. only the console pops up, but not the graphics window), try disabling your antivirus software or add a suitable exception to your antivirus settings.
