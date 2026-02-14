# Stm32f401_ZXSpectrum_Emulator
# STM32F401 VGA Output Demo (Technical Exploration)

This project demonstrates real‑time VGA signal generation on an STM32F401
microcontroller. The firmware produces a stable 640×480 VGA output using
timers, DMA, and carefully timed routines. The goal of this project is to
explore the limits of cycle‑accurate video generation on a Cortex‑M4 device.

## VGA Output
The current build provides a stable VGA signal with clean sync timing and a
working framebuffer. All video‑related code is active and functional in this
repository.

## USB and OSD (Commented Out)
Earlier versions of this project included:
- A USB subsystem
- An on‑screen display (OSD) layer

Both features worked independently, but enabling USB introduced visible tearing
in the VGA output. This behaviour is expected on the STM32F401 due to the
strict real‑time requirements of VGA signal generation.

USB Full‑Speed requires periodic interrupt servicing, and the additional timing
jitter interferes with the video pipeline. As a result:

- **With USB enabled:** OSD works, but the VGA output shows tearing  
- **With USB disabled:** VGA output is fully stable

To prioritise video timing accuracy, the USB and OSD code remains in the
repository for reference but is commented out in the active build.

## Purpose of This Repository
This is not a finished product. It is a technical exploration showcasing:
- Real‑time embedded design
- DMA‑driven video generation
- Timer‑based sync signal creation
- Low‑level ARM Cortex‑M4 firmware
- Hardware debugging with oscilloscope/logic analyser

The project demonstrates the challenges and trade‑offs involved in combining
USB, OSD, and VGA on a mid‑range MCU, and highlights the importance of timing
determinism in embedded video applications.
