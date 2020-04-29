---
title: Benchmarking tooling Tutorial - Shadow Builder
permalink: /docs/tutorials/advanced/benchmarking/
author: Alexandre Malki (improved by Tomasz Kołcon)
---


The Trace Framework Abstraction is a module that will be part of the shadow
builder. This module is in charge of abstracting the underlying tracing tool
used to benchmark.  The tool is still a stiLl a work in progress 
(everything is perfectible). So over time new features and bugs resolving will
be pushed.

## Table of Contents

1. [Introduction](#introduction)
2. [Architecture](#architecture)
3. [Getting started](#getting-started)
4. [Tested platforms](#tested_platform)
5. [Common errors and bugs](#common-errors-and-bugs)

## Introduction

There are some information to know before jumping into the shadow builder. The
architecture section provides a glampse of the tool's guts. But before starting, we need some
terminology to speak the same language.

### Terminology

#### Tags
 A tag is a comment that is use by the shadow-builder. A comment formatted
as follow:

 * /\*\* Benchmarking::ModuleGroup::Function \*/ : without any parameters,
 * /\*\* Benchmarking::ModuleGroup::Function(param0,...,paramN) \*/ : with
   paramaters

is considered a _shadow-builder comment_ also known as a **Tag**.

#### TFA: Trace Framework Abstraction  

This is a Framework provided to aid deloveloper to create various plugin.

#### Plugins

Plugins are library files that are providing replacement for a **Tag**.

#### Parser

The element (front-end) in charge of looking for comment within a string. If
this string comment is **Tag** it's dispatched.

## Architecture

The Shadow-Builder is made of 4 core components that are:

 * **The parser element**: Which is just a backend tool that indicates the TFA
   module that a **Tag** was found in the source code. It's also in charge of
   writing some piece of code. Currently, the llvm and clang are used to parse
   and adapt the code. The use of clang libtooling is documented more in details
   on the official webpage: [https://releases.llvm.org/10.0.0/tools/clang/docs/index.html](https://releases.llvm.org/10.0.0/tools/clang/docs/index.html) .

 * **The TFA module**: This software is in charge of manipulating the different 
   plugins and dispatch the **Tags** found by the the parser
   element. It will perform a first filtering to make sure this comment is 
   formated correctly. In addition it also sanitises the commentaries (also
   called tags). The TFA module undersanding is quite straight forware but more details are provided
   within the [tfa_core/README.md](tfa_core/README.md) .

 * **The TFA plugins**: These are element that answer to a TFA module dispatch
   when a commentary matches the commentaries. The plugins are basically a
   shared library file that will be opened on startup. Some more details how to
   use and write them here: [tfa-plugins/README.md](tfa-plugins/README.md) .

 * **Shadow Builder**: The umbrella core module that is in charge of
   orchestrating the modules aforementionned. More about it here 
   [shadow-builder/README.md](shadow-builder/README.md) . 

### Tree

Below the tree view of the important folders, files and their descriptions.

shadow_builder\
	├── common  		--> Toolbox source files \
	├── examples		--> Examples of instrumented code \
	├── ext			--> External dl and libraries (do not pollute your machine and no need for docker) \
	├── prepare_build.sh    --> Script fetching and installing the dependencies \
	├── res			--> Configuration files folder \
	├── shadow-builder	--> Shadowbuilder source files \
	├── tfa_core		--> TFA's source files, \
	└── tfa-plugins		--> TFA plugin's folder.

## Getting started

### Dependencies

First of all, in order to start compiling the shadow-builder it is needed to
retrieve a list of dependencies. 

#### Under Ubuntu 18.04:

``` shell
sudo apt install git libzmq3-dev binutils libssl-dev python3-distutils python3
sudo snap install cmake --classic # To get a more recent version of cmake.
```

In order to build clang and LLVM, gcc 9 is needed. Under the official ppa, this
is not possible to find it. Therefore, we will need to install from a test ppa:

``` shell
sudo add-apt-repository ppa:ubuntu-toolchain-r/test # Accept the key by pressing enter
sudo apt update
sudo apt install gcc-9 g++-9 g++-9-multilib
```

Then once this is done, as you might already have a gcc on your machine, the 
update-alternative should be used to have different version of gcc. If the
alternative for gcc is available and set up on your machine, you can direclty skip the
command block below.

``` shell
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all cc
sudo update-alternatives --remove-all g++
sudo update-alternatives --remove-all c++

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-xxx 10 # This is your gcc compiler
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-xxx 10 # This is your g++ compiler
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 20

sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
sudo update-alternatives --set cc /usr/bin/gcc

sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++

sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++
``` 

Then select the gcc9 and g++9 as following:

``` shell
sudo update-alternatives --config gcc
There are 2 choices for the alternative gcc (providing /usr/bin/gcc).

  Selection    Path            Priority   Status
------------------------------------------------------------
* 0            /usr/bin/gcc-9   20        auto mode
  1            /usr/bin/gcc-7   10        manual mode
  2            /usr/bin/gcc-9   20        manual mode

Press <enter> to keep the current choice[*], or type selection number: 2
```

And repeat those steps for:
 * c++ --> selecting /usr/bin/g++
 * cc --> selecting /usr/bin/gcc
 * gcc --> selecting /usr/bin/gcc-9
 * g++ --> selecting /usr/bin/g++-9


It is possible to check if the right version of gcc and g++ in the command line:

``` shell
gcc -v
g++ -v
cc -v
c++ -v
```


If some depencies were to be missing, please feel free to add them into this
document and then initiate a pull request.

### Compilation 

Before starting, some internal dependencies must be retrieved. To do so, the
script at the root of folder should be called as follow:

``` shell
./prepare_build.sh # retrieves/compiles and installs dependencies
```

Once this step is done, you can delete the ext/dl subdirectory. This will save you a lot
of space

**This might take some time, go grab yourself a coffee**


Then once all dependencies are built, the shadow-builder is ready to be
compiled:

``` shell
mkdir -p build; cd build
cmake ..
make -j4
```

### First run: using the example/example_stupid_main

To run the the shadow buidler against the examples (located ina examples/example_stupid_main),
the following steps to perform are:

```shell
cd build # previously created during the compilation step.
./shadow-program -s ../res/sb-res/bcf.xml -t ../res/tfa-res/tfa.xmlb
```

This command will create an instrumented code that will be generated in the
folder /tmp/output/test_DATE. 

Once in the folder, the following commands will compile and execute the code:

``` shell
cd /tmp/output/test_DATE/
mkdir -p build; cd build
cmake .. ; make
./simple_stupid_example 
```

The output of this command should be:

``` shell
Monitor var i: 0
Monitor var i: 4294967295
Exe time: 8 sec : 758 ms : 534352 ns
```

To understand what is going, it is recommanded to go through all configurations
file explaination and readmes, the examples code and its readme. 

For more details, refering to all readmes are a good start. The code is
documentated as well and can provide additional information about what is going
on at a very low level.

### Configuration files

The shadow-builder is using XML configuration files to locate the TFA pluging folder 
**currently only one folder can be provided**. By default the plugin folder will
be located in the folder <src_root>/tfa-plugins/

More information regarding the different attributes and node fo the XML
configuraition can be find here: [res/README.md](res/README.md) .

### Plugins

Plugins are the "Responder" to _Tags_ that are found in the code. This _Tags_
are used to tell the shadow-buidler and more specifically its plugins that a
some benchmarking request are made in the code. 

The response is up to the implementation in the plugin. For more details, please
have a look here [tfa-plugins/README.md](tfa-plugins/README.md) .


### Unit tests

Unit tests are available in the folder <module>/tests/xxx_\<xx\>.cpp
Each test shall be added in the CMakeFile.txt in the module you are testing.

In addition the option -DENABLE_TESTS=ON shall be passed to the cmake to
activate tests creation.

Curently a few are test are performed. In the future, more unit test will
be added.

## Tested Platforms

The shadow-builder was successfully compiled with ubuntu 18.04 LTS with the following
configuration:

 * 2 Core / 4 Threads Intel Processor
 * 4GB DDR4 RAM / 8 GB of swapping memory,
 * 60 GB of disk space.

## Common errors and bugs

### Not enough Disk space

Compilations of LLVM and clang are producing a lot of object files et so on.
During the preparation, it is needed around 36GB of disk space.

### Not enough RAM during LLVM/clang compilation

This error might occur during the execution of the script prepare_build.sh

Unfortunately the LLVM/clang compilation is quite greedy when it comes to resources needed
to compile it.

One error arise with low RAM. Currently the only way to solve  would be to increase
the swap size as explained here [https://askubuntu.com/questions/927854/how-do-i-increase-the-size-of-swapfile-without-removing-it-in-the-terminal](https://askubuntu.com/questions/927854/how-do-i-increase-the-size-of-swapfile-without-removing-it-in-the-terminal)



## Going through all the process

This section is dealing with one specific benchmarking tooling called the
Shadow Builder. More specifically, this tutorial aims to create a plugin from
A to Z and how to instrument your code.

For the sake of ease of understanding, this tutorial is proposing to
benchmark the time spent on a simple looping function.

## Prerequisite

Before getting to the heart of the matter, it is needed to meet the following
requirements:

 1. It is assumed that the the **shadow builder** and **trace framework abstraction**
terminology and mechanisms are a known. If this is not the case, the
documentation related to the benchmarking is available in
[here](https://github.com/micro-ROS/benchmarking_shadow-builder/blob/master/README.md).

 2. Using linux, preferably Linux Ubuntu 18.04 and above, all debian based
    distros should do the job.

 3. Some knowledge about c and c++ programing


Once all the checkboxes ticked the tutorial can begin.

## TFA - Plugin

In order to create a plugin, the information that is crucial to figure out are:

What is to benchmark? --> The time spent in a a function.
How to do so ? --> Is there a plugin already supporting it? Yes, then to do. And
the code to profil can be instrumented.

If no plugin supports it, then a plugin has to be created.
Then another set of question arise which are (according to the context):

 1. How could it benifit to many others? 
 2. What piece of code would be used to measure the time? (in C or C++).
 3. What plaform can it support? (OS,CPU etc...).
 4. How the code shall be instrumented?


The answers to this question would be:

 1. Create a generic plugin and write a documentation that would be
    understandable for a normal user and an expert user.
 2. Using the timespec ang clock_gettime Linux syscall.
 3. From previous answer --> OS: Linux any cpu as long as it has the same Linux api.
 4. Using a simple way using the comment as follow 
    /** Benchmarking::plugin_name::function */ . The choice for the current
    tutorial would be /** Benchmarking::TimeBenchmarking::Timer */

 

These answers provided us with the minimun necessary for  the creation of plugin.

# Create a tfa - plugin

## Files tree structure

The final code shall be located in the folder path
src_root_sb/tfa-plugin/timebenchmarking with the following structure:

timebenchmarking
	├── CMakeLists.txt
	├── inc
	│   └── plugin_timebenchmarking
	│       └── plugin_timebenchmarking.h
	└── src
	    └── plugin_timebenchmarking.cpp


## Register a new plugin into the TFA core of the shadow builder

The shadow-builder is relying on tfa's plugins to be executed to answer the
parser dispatch. Therefore the need of some interoperability is needed.

Every new plugins are written by implementing the IPlugin interface as shown in
the file src_root/tfa_core/inc/tfa/IPlugin.h. All what the interface needs to do is to
implement the pure virtual function. A simple example would be as in the
plugin_test:

in the plugin header:

``` cpp
class TimeBenchmarking: public IPlugin {
public:
	TimeBenchmarking();
	~TimeBenchmarking();

	TFAInfoPlugin& getInfoPlugins();
	bool initializePlugin();
private:
};

extern "C" IPlugin* create() {
	return static_cast<IPlugin *>(new TimeBenchmarking);
}
extern "C" void destroy(IPlugin* p) {
	delete p;
}
``` 
in the plugin source code:

``` cpp
TimeBenchmarking::TimeBenchmarking() {}

TimeBenchmarking::~TimeBenchmarking()
{
	if(mInfos) {
		delete mInfos;
	}

}
```

And set the 

```cpp
TFAInfoPlugin& TimeBenchmarking::getInfoPlugins()
{
	return *mInfo;
}

bool TimeBenchmarking::initializePlugin()
{	
	/* This is a plugin compatibility platform */
	tbp = new TFABenchMarkingPlatform("Linux", "*", "*", "*")

	/* This is the infoPlugin that holds the plugin name and the
		platform information */
`	mInfos = new TFAInfoPlugin("Test Plugin", *tbp);

	// Will be explained later how to mock up this part.
	return Status::returnStatusOkay();
}
```

## Create a listener

Good! Now the plugin is ready to be registered within the TFA's core. So when a
session is running, the plugin will be found. However nothing will really
happen. Indeed your plugin is not listening to a specific tag.

Just as a reminder, the listener is an object derivated from the interface
**ITFACommentListener**.  It is listening to as specific _Tag_ which will be
replace by a piece of code.

The declaration of the object shall be as display below:
``` cpp
class Timer: public ITFACommentListener
{
public:
	Timer();
	Status runnableComments(const TFACommentInfo& cleanComment,
				std::string& replacement);
private:
};
```

As shown above, the class is inheriting from the ITFACommentListenner classe. 
the ITFACommentListener as one pure virtual method called runnableComments.
This means your plugin has to implement the method runnableComments(...).


``` cpp

const char difftime_func[] =
"{\n\
	struct timespec *start = &%s;\n\
	struct timespec *stop = &%s;\n\
	struct timespec result;\n\
	if ((stop->tv_nsec - start->tv_nsec) < 0) {\n\
		result.tv_sec = stop->tv_sec - start->tv_sec - 1;\n\
		result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;\n\
	} else {\n\
		result.tv_sec = stop->tv_sec - start->tv_sec;\n\
		result.tv_nsec = stop->tv_nsec - start->tv_nsec;\n\
	}\n\
	#include <stdio.h>\n\
	printf(\"Exe time: %%ld sec : %%ld ms : %%ld ns\\n\",\n\
		       	result.tv_sec, result.tv_nsec / 1000000, result.tv_nsec
%% 1000000);\n\
}\n" ;

Timer::Timer()
 :
	 ITFACommentListener("Benchmarking::User::Timer")
{
}

Status Timer::runnableComments(const TFACommentInfo& cleanComment,
		std::string& replacement)
{
	return Status::returnStatusError();
}
```

Now the functions are correclty implemented. The timer needs several things to
measure the time spent in a function:

 1. Start the timer before the function, get an intial timestamp
 2. Stop the timer after the function has returned, get another timestamp
 3. Measure the delta between the two timestamp measured above.
 4. Print the delta in a human readable.

This basically means that the plugin will neeed a way to get the timestamps, as
discussed before, by using the clock_gettime, and print it to the user by using
printf.

A tag can be provided by several parameters. This will be usefull for the sack of
the timer:

 * A parameter to identify what's is the timer's status (i.e. start or stop)
 * A parameter to identify the timer itself in a unique way by the dev
 * A parameter that is needed for header declaration .

This would look like that in real life without any tools: 

```c

#include <time.h> // Needed to access the clock_gettime() function
#include <stdio.h> // Needed to access the printf function.

void func2benchmark(...)
{

	/** declare and measure the starting timestamp */
	struct timestamp timer_start, timer_stop;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timer_start);
	// do something very slow

	/** Measure the timesampe now, process and show the results */
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timer_start);
	{
		struct timespec *start = &timer_start;
		struct timespec *stop = &timer_stop;
		struct timespec result;

		if ((stop->tv_nsec - start->tv_nsec) < 0) {
			result.tv_sec = stop->tv_sec - start->tv_sec - 1;
			result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
		} else {
			result.tv_sec = stop->tv_sec - start->tv_sec;
			result.tv_nsec = stop->tv_nsec - start->tv_nsec;
		}

		printf(\"Exe time: %ld sec : %ld ms : %ld ns\\n\",
				result.tv_sec, result.tv_nsec / 1000000, result.tv_nsec % 1000000);
	}
}
```

This would llok like this using the tfa:
```c
/** Benchmarking::TimeBenchmarking::Timer(declare) */

void func2benchmark(...)
{
/** Benchmarking::TimeBenchmarking::Timer(start, timer1) */
	//do something very slow
/** Benchmarking::TimeBenchmarking::Timer(stop, timer1) */
}
```

By taking a look at the differences, the amount of overhead code clarity
introduce by the method is very low. 

Now, let's roll-up the sleeves and implement it as a plugin and therefore in a
generic way to be reusable.

### Declare

In order to declare the include needed to benchmark. The way to do it would be
to get the parameter 0 to be a string that matches "declare". The replacement
will be "#include \<time.h\>\n#include \<stdio.h\>\n" 

It is needed to append "\n" to the end of a line, as this piece of code is going
to be appended to the code. 

```cpp

Status Timer::runnableComments(const TFACommentInfo& cleanComment,
		std::string& replacement)
{
	const std::vector<std::string> params = comment.getParams();

	if (params[0] == "declare" && params.size() == 1) {
		replacement = "#include <time.h>\n";
		replacement += "#include <stdio.h>\n";
		return Status::returnStatusOkay();
	}

	return Status::returnStatusError();
}
```

It is mandatory to return Status::returnStatusOkay() to tell the tfa-core that
the _Tag_ was handled and therefore no other plugins will be using it.

#### Start 

The starting element will basically record a timestamp in the memory. How to do
so in C programming on a Linux system would be as follow:

```cpp
Status Timer::runnableComments(const TFACommentInfo& cleanComment,
		std::string& replacement)
{
	const std::vector<std::string> params = comment.getParams();


	if (params[0]  == "start" && params.size() == 2) {
		std::string start = "timer_start_" + params[1]; 
		std::string stop = "timer_stop_" + params[1]; 

		replacement = "struct timespec " + start + ", " + stop + ";\n";
		replacement += "\tclock_gettime(CLOCK_PROCESS_CPUTIME_ID,
			    &timer_start_" + params[1] + ");\n";
		return Status::returnStatusOkay();
	}

	return Status::returnStatusError();
}
```

It is mandatory to return Status::returnStatusOkay() to tell the tfa-core that
the _Tag_ was handled and therefore no other plugins will be using it.

#### Stop

Then the stopping element, which will be in charge of getting a timestamp,
make the delta time spent between the stop and the start and finally print in a
human-readable way.

```cpp

Status Timer::runnableComments(const TFACommentInfo& cleanComment,
		std::string& replacement)
{
	const std::vector<std::string> params = comment.getParams();
	const char difftime_func[] =
	"{\n\
		struct timespec *start = &%s;\n\
		struct timespec *stop = &%s;\n\
		struct timespec result;\n\
		if ((stop->tv_nsec - start->tv_nsec) < 0) {\n\
			result.tv_sec = stop->tv_sec - start->tv_sec - 1;\n\
				result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;\n\
		} else {\n\
			result.tv_sec = stop->tv_sec - start->tv_sec;\n\
				result.tv_nsec = stop->tv_nsec - start->tv_nsec;\n\
		}\n\
		printf(\"Exe time: %%ld sec : %%ld ms : %%ld ns\\n\",\n\
				result.tv_sec, result.tv_nsec / 1000000, result.tv_nsec
				%% 1000000);\n\
	}\n";

	if (params[0]  == "stop" && params.size() == 2) {
		std::string start = "timer_start_" + params[1]; 
		std::string stop = "timer_stop_" + params[1]; 
		char buf[sizeof(difftime_func) + start.length() +
			stop.length()];

		sprintf(buf, difftime_func, start.c_str(), stop.c_str());
		replacement += "clock_gettime(CLOCK_PROCESS_CPUTIME_ID,
			    &timer_stop_" + params[1] + ");\n";
		replacement += string(buf);

		return Status::returnStatusOkay();
	}

	return Status::returnStatusError();
}
```

It is mandatory to return Status::returnStatusOkay() to tell the tfa-core that
the _Tag_ was handled and therefore no other plugins will be using it.

Additionally, this is necessary to think that the replacement code is actual
C code that is going to be compiled. Therefore, one should be careful about the
the way to format it and be careful of the escaping charactere.

#### Combine everything together

Finally the whole runnableComment method will be look like that:

```cpp
Status Timer::runnableComments(const TFACommentInfo& cleanComment,
		std::string& replacement)
{
	const char difftime_func[] =
	"{\n\
		struct timespec *start = &%s;\n\
		struct timespec *stop = &%s;\n\
		struct timespec result;\n\
		if ((stop->tv_nsec - start->tv_nsec) < 0) {\n\
			result.tv_sec = stop->tv_sec - start->tv_sec - 1;\n\
				result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;\n\
		} else {\n\
			result.tv_sec = stop->tv_sec - start->tv_sec;\n\
				result.tv_nsec = stop->tv_nsec - start->tv_nsec;\n\
		}\n\
		printf(\"Exe time: %%ld sec : %%ld ms : %%ld ns\\n\",\n\
				result.tv_sec, result.tv_nsec / 1000000, result.tv_nsec
				%% 1000000);\n\
	}\n";

	const std::vector<std::string> params = comment.getParams();

	if (!params.size())
	{
		return Status::returnStatusError();
	}

	if (params[0] == "declare" && params.size() == 1) {
		replacement = "#include <time.h>\n";
		replacement += "#include <stdio.h>\n";
		return Status::returnStatusOkay();
	} else if (params[0]  == "start" && params.size() == 2) {
		std::string start = "timer_start_" + params[1]; 
		std::string stop = "timer_stop_" + params[1]; 

		replacement = "struct timespec " + start + ", " + stop + ";\n";
		replacement += "\tclock_gettime(CLOCK_PROCESS_CPUTIME_ID,
				&timer_start_" + params[1] + ");\n";
		return Status::returnStatusOkay();
	} else if (params[0]  == "stop" && params.size() == 2) {
		std::string start = "timer_start_" + params[1]; 
		std::string stop = "timer_stop_" + params[1]; 
		char buf[sizeof(difftime_func) + start.length() +
			stop.length()];

		sprintf(buf, difftime_func, start.c_str(), stop.c_str());
		replacement += "clock_gettime(CLOCK_PROCESS_CPUTIME_ID,
					&timer_stop_" + params[1] + ");\n";
		replacement += string(buf);
		
		return Status::returnStatusOkay();
	}

	return Status::returnStatusError();
}
```

## Register the listener

Once the listener is implemented, then it needs to to be registerd within the
plugin:

```cpp
bool TimeBenchmarking::initializePlugin()
{	
	/* This is a plugin compatibility platform */
	tbp = new TFABenchMarkingPlatform("Linux", "*", "*", "*")

	/** Here register the Timer listener */
	iclVect.emplace_back(static_cast<ITFACommentListener *>(new
					Timer));

	/* This is the infoPlugin that holds the plugin name and the
		platform information */
`	mInfos = new TFAInfoPlugin("Test Plugin", *tbp);

	// Will be explained later how to mock up this part.
	return Status::returnStatusOkay();
}
```

A protected vector,from the IPlugin, class needs to be appended for each listener this plugin will be
supporting and implement.


## Compilation files

The compilation file will be the CMakeLists.txt at the root of the plugin

It shall look like the following:

```cmake
cmake_minimum_required(VERSION 3.13)  # CMake version check
project(plugin_timebenchmarking VERSION 0.1 DESCRIPTION "MY Plugin")
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard

set(PLUGIN_NAME "plugin_timebenchmarking")

# Needed to get the function to do tests 
include(../../CMakeMacros/CMakeTesting.txt)

# Plugins include folders
include_directories(inc/)

# Plugins source files
list(APPEND TEST_PLUGIN_SRC
        src/plugin_timebenchmarking.cpp
)

# Needed to create a shareable library
add_library(${PLUGIN_NAME} SHARED ${TEST_PLUGIN_SRC})

# Target library that we need to link against
target_link_libraries(tfa)

# Needed to create a version of the shared library
set_target_properties(${PLUGIN_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
```

And finally, it is needed to add into the parent folder's CMakelists the
subdirectory of the plugin:

```cmake
cmake_minimum_required(VERSION 3.10)  # CMake version check
set(CMAKE_CXX_STANDARD 14) # Enable c++14 standard

add_subdirectory(plugin_test)
add_subdirectory(myplugin)

# Add your project configuration here:
add_subdirectory(plugin_timebenchmarking)
```

### Compilation
To compile the plugin. From the build folder created before in the
shadow-builder.

## Configuration

### TFA configuration

An example fo the configuration file is in the source tree at
src_root/res/tfa-res/tfa.xml

This file only keeps track of the path where to look for plugins. Watch out! this
file is a template and renewed at each compilation. 

In the current context, the path is the default one.


### The shadow builder configuration

The shdow-buidler configuration is providing some hints where the source files
to benchmark can be found and where the ouput folder should be set.

A detailed explaination can be found
[here](https://github.com/micro-ROS/benchmarking_shadow-builder/blob/master/res/README.md#shadow-builder-configuration).

## Running the shadow-builder
Once all the step done above and plugin compiled the command to type woud be:

```shell
cd src_root/build/ 
make
./shadow-program -s ../res/sb-res/bcf.xml -t ../res/tfa-res/tfa.xml
```

The output should be put in the folder configured in the bcf configuration file
(by default should /tmp/output/) under the folder then session's name (test by
default) appended by the date and time when the benchmarking was started.

