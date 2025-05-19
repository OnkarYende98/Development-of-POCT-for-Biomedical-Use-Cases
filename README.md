# Development-of-POCT-for-Biomedical-Use-Cases

This project is designed to detect and analyze chemiluminescent regions using a Raspberry Pi and camera module. The system focuses on identifying blue-colored regions in real-time video frames, calculates their RGB intensity, and displays key metrics on a Nokia 5110 LCD. An LED indicates the system's active monitoring state.

## Features

1) Real-time detection of blue regions using HSV color masking.
2) Calculation of mean RGB values and total intensity within the region of interest (ROI).
3) Identification of the frame with the highest intensity and its brightest pixel.
4) Display of maximum intensity values on a Nokia 5110 LCD screen.
5) LED indicator for active processing state.

## Hardware Requirements

1) Raspberry Pi 4b (with camera support)
2) Raspberry Pi Camera Module
3) Nokia 5110 LCD display (PCD8544 driver)
4) LED with current-limiting resistor (e.g., 330Ω)
5) Jumper wires and breadboard

## Software Requirements

1) Python 3
2) OpenCV
3) NumPy
4) luma.lcd (`pip install luma.lcd`)
5) RPi.GPIO

## Setup Instructions

1. Connect the hardware components:

   * Connect the Nokia 5110 LCD to the SPI interface on the Raspberry Pi.
   * Connect the LED to a GPIO pin with a resistor.
   * Connect the Raspberry Pi Camera Module and enable it in the system settings.
   * Enable SPI via `raspi-config`.

2. Install required Python libraries:
   pip install opencv-python numpy luma.lcd RPi.GPIO

## Output

* Terminal output includes:

  * Maximum detected intensity value
  * Mean RGB values for the highest intensity frame
  * Brightest pixel coordinates and intensity
  * Average intensity of surrounding pixels

* The LCD displays:

  * Maximum intensity (absolute and percentage)

## Cleanup

GPIO is automatically cleaned up at the end of execution to ensure safe shutdown of hardware components.


