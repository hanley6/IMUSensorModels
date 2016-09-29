//
//	main
//	David Hanley
//	
//	main.cpp
//	Entry point to running example sensor model output.
//
//	Options:
//
//
//	Usage:
//		
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
// This is used because Visual Studio has deprecated fopen to make code more 
// secure. Their alternative "fopen_s" is more secure but is not portable, so 
// we simply remove the deprecation.
#define _CRT_SECURE_NO_DEPRECATE 
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include <stdio.h>
#include "SensorModel.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Pragmas -------------------*/
/*---------------- End Pragmas -----------------*/

/*------------- Function Prototypes ------------*/
/*----------- End Function Prototypes ----------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Preamble ---------------------------------*/
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
/*-------------------------------- Helpers ------------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*------------------------------ End Helpers ----------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*----------------------------------- Main ------------------------------------*/
/*-----------------------------------------------------------------------------*/
int main()
{
	/*--------------- Initializations ---------------*/
	// Variable Initializations
	int iter_max = 1522987;
	FILE *outData;
	SensorModel gyro_x;

	// Set sensor parameters
	gyro_x.B = 4.889237695747431e-05;
	gyro_x.K = 1.309017731704734e-06;
	gyro_x.bias = 0.0;
	gyro_x.freq = 98.4163;
	gyro_x.rrw = 0.0;
	gyro_x.sig_meas = 0.009626800281770;
	gyro_x.sig_w = 4.500000000000000;
	gyro_x.T = 1.608951674938202e+03;
	gyro_x.S = 0.0;
	gyro_x.Bf = 0.0;

	// Output File Initializations
	outData = fopen("Simulation_Output.txt", "w");
	fprintf(outData, "Time (sec)	Sensor Measurement\n");

	// Console Output Initializations 
	printf("Begining Sensor Model Simulation...\n");
	/*------------- End Initializations -------------*/

	/*---------------- Run Simulation ---------------*/
	for (int i = 1; i <= iter_max; i++)
	{ 
		// Error-free sensor measurement
		gyro_x.SensorMeasTrue = 0.0;
		// Compute corrupted sensor measuremnt
		gyro_x.SensorModelOutput();
		// Print result to file
		fprintf(outData, "%f\t%f \n",((double)i) / gyro_x.freq - 1.0 / gyro_x.freq,gyro_x.SensorOutput);
		// Print time to console
		printf("Time: %f seconds\r", ((double) i)/ gyro_x.freq - 1.0/ gyro_x.freq);
	}
	/*-------------- End Run Simulation -------------*/

	/*------------------- Closings ------------------*/
	// Closing Console Output
	printf("Time: %f seconds\n", ((double) iter_max) / gyro_x.freq - 1.0 / gyro_x.freq);
	printf("Sensor Model Simulation Complete!\n");

	// Closing File
	fclose(outData);

	return 0;
	/*----------------- End Closings ----------------*/
}
/*-----------------------------------------------------------------------------*/
/*--------------------------------- End Main ----------------------------------*/
/*-----------------------------------------------------------------------------*/