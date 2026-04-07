# ECE532: SPICE Circuit Simulator on FPGA
This repository contains our (System)Verilog and C language code for our ECE532 Project, which involved creating an end-to-end SPICE simulator on a Xilinx FPGA. 

Please refer to the [system introduction video]([https://www.youtube.com/watch?v=Aq5WXmQQooo](https://youtu.be/RuI5V7NdRsY?si=NIAQEJ6qpC3ll3a2)) for a brief overview of the project and final achievements.

The repository is organized in the following structure:
- IP_modules: (System)Verilog files involved in our AXI-wrapped LU Decomposition and Solver IP, alongside testbenches used for system verification.
- software: C code that runs on Microblaze soft-processor for UART input and VGA output
- ip_project: Archived project for final packaged LU Decomposition and Solver IP.
- final_project: Archived project for entire system project used in Final Demonstration
- docs: PDFs of final report and presentation


