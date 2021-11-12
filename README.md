# Contiki OS with InPhase Phase-Based Ranging

This repository contains a Contiki OS fork with the InPhase system integrated into it.

InPhase is implemented for the INGA sensor node. You find the InPhase implementation in `platform/inga/dev/distance-sensor`.

An example usage of the software is provided in `examples/inphase/simple_ranging.c`. It uses two sensor nodes and measures the distance between them in an infinite loop.
