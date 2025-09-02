# Rocket-Avionics
Embedded firmware for Rocket Avionics

Read Me last updated: 9/2/2025

## Note:
This is a mono-repo meaning that code for the bay boards, flight computer, and all other rocket avionics should go into sub-directories of this repo. The only exception to this is Limewire since that is an interface to the flight computer and not part of rocket avionics itself.

Find all firmware that actually runs on boards in `stm-flight-sw/`

Find all the templates in `stm-templates/`

Sensor Emulation Board (SEB) -> `SEB/`

Apogee Detection Sims & RTOS Threads -> `apogee-detection/`

All on-board driver firmware -> `board-drivers/`