# PYNQ-PRIO

This repository, created by the BYU Configurable Computing Lab, contains the pip install package for the Partially-Reconfigurable Input/Output (PRIO) Project on the PYNQ.

This project demonstrates the use of partial reconfiguration with the PYNQ platform. It contains demonstrations on the software API for downloading and managing partially-reconfigurable regions with the pynq package, as well as example hardware projects for the PYNQ-Z1, PYNQ-Z2 and the ZCU104 boards that implement partially reconfigurable regions.


## Quick Start

In order to install it on your PYNQ, you're PYNQ must be running the v2.4 image. The most up to date PYNQ images can be found <a href="http://pynq.io" target="_blank">here</a>. See the <a href="http://pynq.readthedocs.io/en/latest/getting_started.html" target="_blank">Quickstart guide</a> for details on writing the image to an SD card.

To install the PRIO project, run the following command from a terminal connected to your board:

```console
sudo -H pip3 install git+https://github.com/byuccl/PYNQ-PRIO.git
```

The pip install will install the prio package to your pynq board. A prio directory
with example notebooks will be created in the ~/jupyter_notebooks directory. These
examples will show you how to use partial reconfiguration with the PYNQ.

## Board Files and Overlays

All board related files including Vivado projects, bitstreams, and example notebooks, can be found in the `/boards` folder.

In Linux, you can rebuild the overlay by running *make* in the corresponding overlay folder (e.g. `/boards/Pynq-Z1/prio`). In Windows, you need to source the appropriate tcl files in the corresponding overlay folder.
