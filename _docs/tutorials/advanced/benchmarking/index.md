---
title: Benchmarking with the Shadow-Builder
author: amalki
permalink: /docs/tutorials/advanced/benchmarking/
redirect_from:
  - /benchmarking/
---

<img src="https://img.shields.io/badge/Applies_to-all_current_distros-green" style="display:inline"/>

- [Benchmarking an applicastion with the TFA Shadow-builder](#benchmarking-an-applicastion-with-the-tfa-shadow-builder)
- [Prerequisites](#prerequisites)
- [TFA Plugin](#tfa-plugin)
- [Configuration](#configuration)
- [Running the shadow-builder](#running-the-shadow-builder)


## Benchmarking an applicastion with the TFA Shadow-builder

This section is dealing with one specific benchmarking tooling called the
Shadow Builder. More specifically, this tutorial aims to create a plugin from
A to Z and how to instrument your code.

For the sake of ease of understanding, this tutorial is proposing to
benchmark the time spent on a simple looping function.

## Prerequisites

Before getting to the heart of the matter, it is needed to meet the following
requirements:

 1. It is assumed that the **shadow builder** and **trace framework abstraction**
terminology and mechanisms are a known. If this is not the case, the
documentation related to the benchmarking is available in
[here](https://github.com/micro-ROS/benchmarking_shadow-builder/blob/master/README.md).

 2. Using linux, preferably Ubuntu 18.04 and above, all Debian-based
    distros should do the job.

 3. Some knowledge about C and C++ programming


Once all the checkboxes ticked the tutorial can begin.

## TFA Plugin

In order to create a plugin, the information that is crucial to figure out are:

What is to benchmark? --> The time spent in a a function.
How to do so ? --> Is there a plugin already supporting it? Yes, then to do. And
the code to profil can be instrumented.

If no plugin supports it, then a plugin has to be created.
Then, another set of questions arises, which are (according to the context):

 1. How could it benefit to others? 
 2. What piece of code would be used to measure the time? (In C or C++?)
 3. What platform can it support? (OS, CPU, etc.)
 4. How should the code be instrumented?


The answers to these questions would be:

 1. Create a generic plugin and write a documentation that would be
    understandable for a normal user and an expert user.
 2. Using the `timespec` and `clock_gettime` Linux syscall.
 3. From previous answer --> OS: Linux on any CPU as long as it has the same Linux API.
 4. Using a simple way using the comment as follow 
   ` /** Benchmarking::plugin_name::function */` . The choice for the current
    tutorial would be `/** Benchmarking::TimeBenchmarking::Timer */`

 

These answers provide us with the minimum necessary for the creation of a plugin.

## Create a TFA-Plugin

### File tree structure

The final code shall be located in
`src_root_sb/tfa-plugin/TimeBenchmarking` with the following structure:

```
TimeBenchmarking
	├── CMakeLists.txt
	├── inc
	│   └──TimeBenchmarking
	│       └── TimeBenchmarking.h
	└── src
	    └── TimeBenchmarking.cpp
```


### Register a new plugin into the TFA core of the shadow builder

The shadow-builder is relying on TFA's plugins to be executed to answer the
parser dispatch. Therefore, the need of some interoperability is needed.

Every new plugins are written by implementing the IPlugin interface as shown in
the file `src_root/tfa_core/inc/tfa/IPlugin.h`. All what the interface needs to do is to
implement the pure virtual function. A simple example would be as in the
plugin_test:

In the plugin header:

```cpp
class TimeBenchmarking: public IPlugin {
public:
	TimeBenchmarking();
	~TimeBenchmarking();

	TFAInfoPlugin& getInfoPlugins();
	bool initializePlugin();
};

extern "C" IPlugin* create() {
	return static_cast<IPlugin *>(new TimeBenchmarking);
}
extern "C" void destroy(IPlugin* p) {
	delete p;
}
``` 
In the plugin source code:

```cpp
TimeBenchmarking::TimeBenchmarking() {}

TimeBenchmarking::~TimeBenchmarking()
{
	if(mInfos) {
		delete mInfos;
	}

}
```

### Create a listener

Good! Now the plugin is ready to be registered within the TFA's core. So when a
session is running, the plugin will be found. However nothing will really
happen. Indeed your plugin is not listening to a specific tag.

Just as a reminder, the listener is an object derivated from the interface
**ITFACommentListener**. It is listening to as specific _Tag_ which will be
replace by a piece of code.

The declaration of the object shall be as display below:
```cpp
class Timer: public ITFACommentListener
{
public:
	Timer();
	Status runnableComments(const TFACommentInfo& cleanComment,
				std::string& replacement);
};
```

As shown above, the class is inheriting from the ITFACommentListenner classe. 
The ITFACommentListener has one pure-virtual method called runnableComments.
This means your plugin has to implement the method runnableComments(...).


```cpp
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

Now, the functions are correctly implemented. The timer needs several things to
measure the time spent in a function:

 1. Start the timer before the function, get an intial timestamp
 2. Stop the timer after the function has returned, get another timestamp
 3. Measure the delta between the two timestamp measured above.
 4. Print the delta in a human readable.

This basically means that the plugin will neeed a way to get the timestamps, as
discussed before, by using the clock_gettime, and print it to the user by using
printf.

A tag can be provided by several parameters. This will be useful for the sake of
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

	if (params.size() == 1 && params[0] == "declare") {
		replacement = "#include <time.h>\n";
		replacement += "#include <stdio.h>\n";
		return Status::returnStatusOkay();
	}

	return Status::returnStatusError();
}
```

It is mandatory to return Status::returnStatusOkay() to tell the tfa-core that
the _Tag_ was handled and therefore that no other plugin will be using it.

#### Start 

The starting element will basically record a timestamp in the memory. How to do
so in C programming on a Linux system would be as follow:

```cpp
Status Timer::runnableComments(const TFACommentInfo& cleanComment,
		std::string& replacement)
{
	const std::vector<std::string> params = comment.getParams();


	if (params[0] == "start" && params.size() == 2) {
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
the _Tag_ was handled and therefore that no other plugin will be using it.

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
				result.tv_sec, result.tv_nsec / 1000000, result.tv_nsec\n\
				%% 1000000);\n\
	}\n";

	if (params[0] == "stop" && params.size() == 2) {
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
the _Tag_ was handled and therefore that no other plugin will be using it.

Additionally, this is necessary to think that the replacement code is actual
C code that is going to be compiled. Therefore, one should be careful about the
the way to format it and be careful of the escaping characters.

#### Combine everything together

Finally the whole runnableComment method will look like that:

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
				result.tv_sec, result.tv_nsec / 1000000, result.tv_nsec \n\
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
	} else if (params[0] == "start" && params.size() == 2) {
		std::string start = "timer_start_" + params[1]; 
		std::string stop = "timer_stop_" + params[1]; 

		replacement = "struct timespec " + start + ", " + stop + ";\n";
		replacement += "\tclock_gettime(CLOCK_PROCESS_CPUTIME_ID, \
				&timer_start_" + params[1] + ");\n";
		return Status::returnStatusOkay();
	} else if (params[0] == "stop" && params.size() == 2) {
		std::string start = "timer_start_" + params[1]; 
		std::string stop = "timer_stop_" + params[1]; 
		char buf[sizeof(difftime_func) + start.length() +
			stop.length()];

		sprintf(buf, difftime_func, start.c_str(), stop.c_str());
		replacement += "clock_gettime(CLOCK_PROCESS_CPUTIME_ID, \
					&timer_stop_" + params[1] + ");\n";
		replacement += string(buf);
		
		return Status::returnStatusOkay();
	}

	return Status::returnStatusError();
}
```

#### Register the listener

Once the listener is implemented, then it needs to be registered within the
TFA plugin manager:

```cpp
bool TimeBenchmarking::initializePlugin()
{	
	/* This is a plugin compatibility platform */
	tbp = new TFABenchMarkingPlatform("Linux", "*", "*", "*");

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

A protected vector, inherited from the IPlugin, class needs to be appended for each listener this plugin will be
supporting and implementing.


### Compilation files

The compilation file will be the CMakeLists.txt at the root of the plugin

It shall look like the following:

```cmake
project(TimeBenchmarking VERSION 0.1 DESCRIPTION "MY Plugin")
set(CMAKE_CXX_STANDARD 14)

set(PLUGIN_NAME "TimeBenchmarking")

# Needed to get the function to do tests 
include(../../CMakeMacros/CMakeTesting.txt)


# Plugins include folders
include_directories(inc/)

# Plugins source files
list(APPEND TEST_PLUGIN_SRC
        src/TimeBenchmarking.cpp
)

# Needed to create a shareable library
add_library(${PLUGIN_NAME} SHARED ${TEST_PLUGIN_SRC})

# Target library that we need to link against
target_link_libraries(tfa)

# Needed to create a version of the shared library
set_target_properties(${PLUGIN_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
```

And finally, it is needed to add into the parent's folder (i.e.
src_root/tfa-plugins/CMakeLists.txt) CMakeLists.txt the
subdirectory of the plugin:

```cmake
cmake_minimum_required(VERSION 3.10)  # CMake version check
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(plugin_test)
add_subdirectory(myplugin)

# Add your project configuration here:
add_subdirectory(TimeBenchmarking)
```

The example is available [here](https://github.com/micro-ROS/benchmarking_shadow-builder/blob/master/tfa-plugins/TimeBenchmarking/)

### Compilation
To compile the plugin. From the build folder created before in the
shadow-builder.

## Configuration

### TFA configuration

An example fo the configuration file is in the source tree at
`src_root/res/tfa-res/tfa.xml`.

This file only keeps track of the path where to look for plugins. Watch out! this
file is a template and renewed at each compilation. 

In the current context, the path is the default one (i.e. src_root/build):


```shell
cd src_root/build/ 
make -j4
```



### The shadow builder configuration

The shadow-buidler configuration is providing some hints where the source files
to benchmark can be found and where the ouput folder should be set.

A detailed explaination can be found
[here](https://github.com/micro-ROS/benchmarking_shadow-builder/blob/master/res/README.md#shadow-builder-configuration).

## Running the shadow-builder
Once all the above steps are done and the plugin compiled the command to run the
code's instrumentation would be:

```shell
cd src_root/build/ 
./shadow-program -s ../res/sb-res/bcf.xml -t ../res/tfa-res/tfa.xml
```

The output should be put in the folder configured in the bcf configuration file
(by default should /tmp/output/) under the folder then session's name (test by
default) appended by the date and time when the benchmarking was started.
