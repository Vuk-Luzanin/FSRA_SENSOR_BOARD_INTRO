# STM32 Project: Digital Inputs, ADC, and Accelerometer

This project implements the following functionalities using an STM32 microcontroller:  
1. Reading a digital signal to activate an LED.  
2. Simultaneous reading of multiple ADC channels to measure values from a **thermistor** and **potentiometer**.  
3. Reading linear acceleration values from the **Compass Click** board, equipped with the LSM303DLHC chip.

---

## Features
1. **Digital Signal and LED:**  
   - The microcontroller reads a digital input signal.  
   - If the signal is active (HIGH), the LED is turned on.  

2. **ADC Measurements:**  
   - Simultaneous reading from two ADC channels:  
     - **Channel 1:** Thermistor for temperature measurement.  
     - **Channel 2:** Potentiometer for analog input simulation.  
   - Results are sent to a serial terminal via UART for visualization.  

3. **Accelerometer (LSM303DLHC):**  
   - Reads acceleration in the X, Y, and Z axes.  
   - Results are displayed in "m/s^2" on the serial terminal.  

---

## Setup
### Hardware Configuration
- Connect the thermistor and potentiometer to the appropriate ADC channels.  
- Pin assignments are specified in the `.ioc` file, which can be opened using STM32CubeMX.  
- Connect the Compass Click board (LSM303DLHC) to the I2C interface.  
- Connect an LED to a designated GPIO pin.  
- Ensure the system is powered and a serial terminal is set up for UART communication.  

### Software Configuration
1. Install **STM32CubeIDE**.  
2. Open the project in STM32CubeIDE.  
3. Build and flash the code to the STM32 microcontroller.  
4. Open a serial terminal (e.g., PuTTY, Tera Term, Arduino, HTerm...) with a baud rate of 115200 bps.  

---

## Running the Project
1. Power up the STM32 NUCLEO-F042K6 development board.  
2. Observe the following outputs on the serial terminal:  
   - Status of the digital signal and LED.  
   - ADC values from the thermistor and potentiometer.  
   - Acceleration values in X, Y, and Z axes.  
