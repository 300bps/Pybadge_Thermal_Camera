### Pybadge Thermal Image Camera Firmware:

This Arduino firmware is a modification and feature enhancement of the original Adafruit “Pybadge\_Thermal\_Image\_Recorder” firmware project originally written by Anne Barela in 2020. Adafruit has a complete project guide covering the parts list, 3D printed case, and hardware build at [https://learn.adafruit.com/pybadge-thermal-camera-case](https://learn.adafruit.com/pybadge-thermal-camera-case). Another Adafruit project guide covers the operation of the original firmware at: [https://learn.adafruit.com/mlx90640-thermal-image-recording](https://learn.adafruit.com/mlx90640-thermal-image-recording). These guides describe the creation of a fully-fledged thermal imaging camera.

### Original Features:

The original firmware has a very nice and extensive feature set. Some of these features include user selectable frame rate, multiple color palettes, image freeze, image save, temperature range selection, temperature auto-ranging, and continuous display of the warmest, coldest, and center temperature.

### Modifications/Enhancements:

The modifications that I made to the original firmware are described here:

*   **4X Interpolation:** I added bilinear interpolation of the 32 element x 24 element temperature data produced by the MLX90640 thermal image sensor. The interpolation produces a 64 element x 48 element temperature array that is displayed for all except the very fastest frame rate. In other words, this increases the raw number of thermal “pixels” from 768 to 3072. This interpolation results in smoother transitions between the measured temperatures and produces an image with an effectively higher visual resolution.  
*   **"Predator-vision" Color Palette:** I added an additional color palette that reminded me of the thermal vision seen in the "Predator" movie. I used the values for this color palette from another Adafruit project at: [https://learn.adafruit.com/thermal-camera-with-display/software.](https://learn.adafruit.com/thermal-camera-with-display/software.)
