# path-following-car
- class project for the UCLA course "Intro to Electrical Engineering".
- created an Arduino robot car that follows a path that has been printed on paper until it reaches the end of the track.
It performs a 180 degree turn at the end of the track, then follows the path back to the start position and haults.
- hardware implementation: photoresistors modulate capacitor discharge rates, which can be monitored by the software to determine
the position of the car relative to the track. Left and right motor speeds can be adjusted accordingly.
- [code](https://github.com/jpicchi18/path-following-car/tree/main/code): C++ program that controls the car.
- [data](https://github.com/jpicchi18/path-following-car/tree/main/data): Sensor data used to design a model for weighted sensor fusion calculations in the software.
- [final_report](https://github.com/jpicchi18/path-following-car/tree/main/final_report): software design overview, prototype logs, 
sensor fusion calculation model, and performance analysis.
  - written using Latex
