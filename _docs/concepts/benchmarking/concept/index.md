---
title: Concepts
redirect_from: /benchmarking/concept/
permalink: /docs/concepts/benchmarking/concept/
---

## Table of contents

* [Introduction to Benchmarking](#introduction-to-benchmarking)
* [Our benchmarking tool framework](#our-benchmarking-tool-framework)
* [Trace Framework Abstraction](#trace-framework-abstraction)
* [Shadow Builder](#shadow-builder)
* [Binary generation for instrumented code](#binary-generation-for-instrumented-code)
  * [Receiving inputs](#receiving-inputs)
  * [Parse and Check](#parse-and-check)
  * [TFA Execution](#tfa-execution)
  * [Compilation](#compilation)
* [Step to start benchmarking](#step-to-start-benchmarking)



## Introduction to Benchmarking

Developing working and stable application from the scribbles to the final
executing binary is long and hard tasks. During this process developers may come
across stabilities issues, perfomances issues. In addition to these issues, some
specified QoS might be difficult to quantify.  Solving those problems without the
proper tools might be frustrating, tedious tasks leading to reduce developers
efficiency. An adapted benchmarking tool could overcome all those development
obstacles and increase development time.  There are different KPI (Keep
Performance Indicators) that one might be interested into. In the framework of
this micro-ROS, the KPI can be freely chosen by the developer. In this way, the
benchmarking tool will remain flexible and allow the community to constantly add
some support for a lot of different KPI.

The problems we want to tackle are: 

 * Out there, many benchmarking tools exist. Each of targeting different KPIs. 
 * Different platforms (Linux/Nuttx/Baremetal et.c.).
 * Too few time/resources to code benchmarking tool for each.
 * Avoid code overhead: Keep code clarity.
 * Avoid execution overhead: Do not want to make execution slower when benchmarking.

## Our Benchmarking tool framework

The benchmarking tool under development is providing a framework to allow
developers to create their own benchmarking tool. Each part a developer wants to
benchmark can be added as a plugin using the provided framework. In this way
plugins can be shared and this improves re-usability as much as possible.


## Trace Framework Abstraction

The Shadow builder alone only parse comments from the application and pass it
along to the Trace Framework Abstraction (TFA) Core. The TFA core is aware of
the plugins that are available, all the plugins’ capabilities and platform
target. The process goes as explained below: 

 * The line containing the functionality Benchmarking::XX::YY will be checked
   against all the available plugins.
 * Plugins that are capable of handling functionality will respond with a piece of
   code that will be replaced with a piece of code.
 * Then the output file will be added in a folder corresponding to the platform
   type and benchmarking type.

Being generic is the key for this benchmarking tool. The plugins will in
contrary bring the specific implementation needed to benchmark  a specific
platform. Every plugin will provide information as  requested by the parser:

 * Provide a list of supported platforms. 
 * Provide a list of functions that are handled.
 * Provide snippets codes that will be added for benchmarking.
 * Provide a list of patches and/or patch code
 * Optional provide an end script to run and execute the benchmarks


## Shadow Builder

This section will introduce some concept related to the shadow builder (SB).

The Shadow builder is a tool that will transparently instrument the code to
benchmark. The tools will be able to output an “instrumented code” that will be
later be compiled as a normal code. The following steps describe what the shadow
builder process flow:

 * Get configuration file from the user (Benchmarking Configuration File).
 * Get appropriate sources.
 * Execute Trace Framework Abstraction Configuration file.
 * Parse the sources file needed Injecting code.
 * Compile the targeted binary for different platform.
 * If needed, depending what type benchmark is undertaken, compile another
   target binary benchmarking.

The SB (Shadow Builder) is meant to be as transparent as possible for the user.
And if the benchmarking is not activated, it should be bypassed.  Get
appropriate sources

The SB is in charge of getting the path/git repository to the source code that
needs to be benchmarking. The benchmarking. The sources are specified by the
user in the benchmarking configuration file.  Injecting code

In order to inject code, there are some tools that allow this. CLang AST tool
will allow to inject some code. 


## Binary generation for instrumented code 

The binary generation is the process of compiling the source code. In order to
benchmark, previously to compile the source code, it is necessary to instrument
the code. The code will be instrumented in a transparent way for the
programmer/user. Therefore, a configuration file provided by the programmer will
be parsed and code injected as described in a configuration file. 

### Receiving inputs

The binary generation's pipeline receives two inputs to work with:
 * Configuration Benchmarking file.
 * Source code to benchmark.

In short, the configuration describes:

 * What is benchmarked (sources).
 * Where to benchmark.
 * What type of benchmark.
 * Optionally against what base line to compare (base line source)

### Parse and Check

Once the input received the **Shadow Builder** parses the configuration
file. From the configuration file, the Shadow builder gets:

 * The different benchmarking to be achieved.
 * The targeted platforms.

In addition to parsing, the Shadow Builder is in charge of checking
capabilities and consistency within the configuration file and the different
TFA's plugins registered in the TFA module.

### TFA Execution

Once parsed and checked against the TFA module capabilities, the Shadow
Builder will be in charge of translating configuration into source code. The
translated sources will also be achieved in cooperation with the TFA module. The
detailed steps of the TFA can be found here. At the end of this step, the TFA
will generate the new forged source code ready for compilation. In addition to
patched source code, the TFA will generate scripts that will the benchmarks.

### Compilation

The compilation will happen for every kind of benchmarks and
platforms targeted. Depending on the kind of benchmark that is being executed,
there will be one or more binaries per benchmarks session. The number of binary
generated also depends on what plugins are provided by the user to the shadow
builder. The shadow builder will retrieve capabilities of the plugins and
request from the developer, match them and generated software according to the
matches.


## Step to start benchmarking

The shadow Builder will be executed as follow:

 * Software sources are passed to the Shadow Builder.
 * The source are passed and upon comments containing /*Benchmarking::XX::YY*/
   (a tag)  the code line is passed to the Trace Framework Abstraction module.
   Using comments is preferable → No includes needed.
 * All plugins that registered to the TFA the Benchmarking::XX::YY functionality
   will return a piece of code that will be added to the source.
 * Once all parsed, the shadow builder will compile for all the different
   platforms requested either by plugins or by user configuration.

