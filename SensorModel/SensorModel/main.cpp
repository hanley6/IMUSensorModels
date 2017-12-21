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
	gyro_x.SetB(4.889237695747431e-05);
	gyro_x.SetK(1.309017731704734e-06);	
	gyro_x.Setbias(0.0);
	gyro_x.Setfreq(98.4163);
	gyro_x.Setrrw(0.0);
	gyro_x.Setsig_meas(0.009626800281770);
	gyro_x.Setsig_w(10.0);
	gyro_x.SetT(1.608951674938202e+03);
	gyro_x.SetS(0.0);
	gyro_x.SetBf(0.0);

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
		gyro_x.SetSensorMeasTrue(0.0);
		// Compute corrupted sensor measuremnt
		gyro_x.SensorModelOutput();
		// Print result to file
		fprintf(outData, "%f\t%f \n",((double)i) / gyro_x.Getfreq() - 1.0 / gyro_x.Getfreq(),gyro_x.GetSensorOutput());
		// Print time to console
		printf("Time: %f seconds\r", ((double) i)/ gyro_x.Getfreq() - 1.0/ gyro_x.Getfreq());
	}
	/*-------------- End Run Simulation -------------*/

	/*------------------- Closings ------------------*/
	// Closing Console Output
	printf("Time: %f seconds\n", ((double) iter_max) / gyro_x.Getfreq() - 1.0 / gyro_x.Getfreq());
	printf("Sensor Model Simulation Complete!\n");

	// Closing File
	fclose(outData);

	return 0;
	/*----------------- End Closings ----------------*/
}
/*-----------------------------------------------------------------------------*/
/*--------------------------------- End Main ----------------------------------*/
/*-----------------------------------------------------------------------------*/