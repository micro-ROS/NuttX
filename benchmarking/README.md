Benchmarking
============

The benchmarking is a tool that provide run-time information regarding
application performances. Currently the tool is only providing information
regarding the heap memory allocation.


How it works?
=============

The benchmarking tool make use of the ITM trace debug facility available in most
of ARM processor. The malloc is instrumented to use the ITM facility to write
backtrace (hierarchic calls) size of allocation and pointer where the memory is
allocated. 

In this way, every call to malloc (user allocation and kernel allocation) are
logged to the ITM software.

Preliminaries
=============

In order to make us of the ITM, the cpu must configured to output ITM trace to
SWD/JTAG output depending on the need. The ARM CoreSight debug IP from must be
configured. This can be done using the OpenOCD tool to write registers remotely
or in the embedded software.

This README will not provide indepth information regarding configuration of the
CoreSight ARM IP. More information can be find CPU's Technical Reference Manual
and ARM Reference Manuals.

How to enable it
================

Enabling options were added to the menuconfig:
	Benchmarking Support --> [ ] Heap memory benchmarking:w


Restrictions
============

Currently this tool is only available for ARM processors. 
Also It was tested only on cortex-M4 processor (STM32F407ZG).
