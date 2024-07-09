# Vertigo
This is an electronic geocache. It is a little PCB in a preform tube ("Petling"). The geocacher is supposed to take the tube and hold it upright on a stretched-out arm. Then, the geocacher needs to turn around five times, one rotation per 2 seconds. Afterward, the code for a lock (or any other clue) is revealed. An example is this geocache: [Vertigo](https://www.geocaching.com/geocache/GC63FC7_vertigo).

You can build it from a [small PCB that fits into a preform tube](https://github.com/felias-fogg/PETPreformBoard). You need an ATtiny1634 in a SOIC-20 package, a 5x7 dot matrix display, and a breakout board with an MPU6050, e.g., the GY-521. 

On the software side, you need a few libraries (which you all can install using Arduino's library manager):

* [DotMatrix5x7](https://github.com/felias-fogg/DotMatrix5x7),
* [Vcc](https://github.com/felias-fogg/Vcc),
* [SoftI2CMaster](https://github.com/felias-fogg/SoftI2CMaster).

In addition, you need to install the [MPU6050](https://github.com/ElectronicCats/mpu6050)  library (also part of Arduino's library manager). The example folder contains the IMU_Zero sketch, which you must use to calibrate the MPU6050 exemplar.

