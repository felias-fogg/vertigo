# Vertigo
This is an electronic geocache. It is a little PCB in a preform tube ("Petling"). The geocacher is supposed to take the tube and hold it upright on a stretched out arm. Then the geocacher needs to turn around five times with one rotation per 2 seconds. Afterwards, the code for a lock (or any other clue) is revealed. An example is this geocache: [Vertigo](https://www.geocaching.com/geocache/GC63FC7_vertigo).

You can build it from a [small PCB that fits into a preform tube](https://github.com/felias-fogg/PETPreformBoard). You need an ATtiny1634 in a SOIC-20 package, a 5x7 dot matrix display, and a breakout board with a MPU6050, e.g. the GY-521. 

On the software side, you need a few libraries (which you all find using Arduino's library manager):

* DotMatrix5x7,
* Vcc,
* SoftI2CMaster.

In addition, you need to install the library MPU6050 (also part of Arduino's library manager), which contains in the `example` folder the `IMU_Zero` sketch, which one has to use to calibrate the MPU6050 exemplar.

