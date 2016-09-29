# IMUSensorModels
Written by: David Hanley

This repository contains a C++ class for IMU sensor modeling. 

This models both deterministic and stochastic elements of individual elements of an IMU (i.e. individual accelerometers and gyroscopes).
Cross coupling effects, g-sensitive bias, and vibro-pendulous errors are ignored since the input is only the error free sensor.

Deterministic modeling is based on:

D. Titterton and J. Weston. "Strapdown Inertial Navigation Technology." The Institution of Engineering and Technology. 2nd Edition. 2004.

Stochastic modeling is based on:

P. Petkov and T. Slavov. "Stochastic Modeling of MEMS Inertial Sensors." Cybernetics and Information Technologies. Vol. 10. No. 2. pp. 31-40. 2010.



Important Folders:

	SensorModel => Example C++ function, main.cpp, using class using SensorModel.h and SensorModel.cpp to simulate a gyroscope (In Visual Studio)

	Matlab SensorModel => Matlab versions of the C++ SensorDetModel and SensorStocModel functions

	Matlab Example => Example of using and analyzing Matlab sensor model