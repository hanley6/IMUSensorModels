//
//	SensorModel
//	David Hanley
//	
//	SensorModel.cpp
//	Implementation file of SensorModel class. This includes a public function, SensorModelOutput,
//	which first finds the deterministic contributors to the sensor error (found using 
//	SensorDetModel) and then finds the stochastic contributors to the sensor error (found using 
//	SensorStocModel). The result is assigned to the public variable, SensorOutput.
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
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include "SensorModel.h"
#include <math.h>
#include <stdlib.h>
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
/*------------------------ randn -------------------------*/
// Generates random number from a normal distribution with mean, mu,
// and standard deviation, sigma. Code from site below:
//https://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
double randn(double mu, double sigma)
{
	/*--------------- Initializations ---------------*/
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;
	/*------------- End Initializations -------------*/

	// The flag call determines if the call to the function
	// randn is even or odd. If call = 0 then we generate two 
	// random numbers from normal distribution with mean 0 and 
	// standard deviation 1 using the Polar method, and then 
	// transform the generated random variable to make it have 
	// a mean mu and standard deviation sigma then return X1. 
	// If call = 1 then we do not compute anything and return 
	// the second normal random number (after mu sigma transformation) 
	// X2 generated in the previous call.
	if (call == 1)
	{
		call = !call;
		return (mu + sigma * (double)X2);
	}

	do
	{
		U1 = -1 + ((double)rand() / RAND_MAX) * 2;
		U2 = -1 + ((double)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	mult = sqrt((-2 * log(W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

	call = !call;

	return (mu + sigma * (double)X1);
}
/*---------------------- End randn -----------------------*/

/*----------------- Default Constructor ------------------*/
SensorModel::SensorModel()
{
	Choice = 1;
}
/*--------------- End Default Constructor ----------------*/

/*--------------- Parameterized Constructor --------------*/
SensorModel::SensorModel(int Kalibr)
{
	Choice = Kalibr;
}
/*------------- End Parameterized Constructor ------------*/

/*-------------------- SensorDetModel --------------------*/
// Outputs corrupted sensor measurement due to deterministic errors. This 
// approach ignore cross coupling effects, g-sensitive bias, and vibro-pendulous 
// error since the input is only the error free sensor output. This approach is 
// based on:
// D. Titterton and J. Weston. "Strapdown Inertial Navigation Technology." The 
// Institution of Engineering and Technology. 2nd Edition. 2004.
void SensorModel::SensorDetModel(void)
{
	// Compute corrupted sensor measurement due to deterministic errors
	DetSensorOutput = (1.0 + S)*SensorMeasTrue + Bf;
}
/*------------------ End SensorDetModel ------------------*/

/*-------------------- SensorStocModel -------------------*/
// Outputs a sensor measurement corrupted by a noise model. 
// This models only the stochastic component of the sensor.Deterministic
// errors, such as scale and turn - on bias, should be applied to the
// sensordata input prior to using this function.The following paper is used
// as a reference for this function:
// P.Petkov and T.Slavov. "Stochastic Modeling of MEMS Inertial Sensors."
// Cybernetics and Information Technologies.Vol. 10. No. 2. pp. 31 - 40.
// 2010.
void SensorModel::SensorStocModel(void)
{
	/*--------------- Initializations ---------------*/
	double sensor_noise;
	double arw;
	double sig_arw;
	double dt;
	double a_d;
	double b_d;
	double sig_eta;
	double eta;

	// Period of sensor measurements
	dt = 1.0 / freq;
	/*------------- End Initializations -------------*/

	/*-------------- Bias Instability ---------------*/
	// Set up discrete time bias instability dynamics
	a_d = exp( -dt / T );
	b_d = T - T*exp( - dt / T );
	sig_eta = B*sqrt( 1 - pow(a_d,2.0) )/b_d;
	eta = sig_eta*randn(0.0, 1.0);
	// Compute next bias instability state
	bias = a_d*bias + b_d*eta;
	/*------------ End Bias Instability -------------*/

	/*--------------- Rate Random Walk --------------*/
	rrw = rrw + K*dt*sig_w*randn(0.0,1.0);
	/*------------- End Rate Random Walk ------------*/
	
	/*-------------- Angle Random Walk --------------*/
	// Compute the standard deviation of the angle random walk
	sig_arw = sqrt(pow(sig_meas,2.0) - pow(K*dt*sig_w,2.0) - pow(B,2.0));
	// Find angle random walk contribution
	arw = sig_arw*randn(0.0, 1.0);
	/*------------ End Angle Random Walk ------------*/

	/*----------- Compute Sensor Output -------------*/
	// Compute Total Sensor Noise Contribution
	sensor_noise = bias + arw + rrw;
	// Compute Total Sensor Measurement
	SensorOutput = DetSensorOutput + sensor_noise;
	/*--------- End Compute Sensor Output -----------*/
}
/*------------------ End SensorStocModel -----------------*/

/*----------------- SensorStocModel_Kalibr ---------------*/
// Outputs a sensor measurement corrupted by a noise model. 
// This model is a reduced version of the SensorStocModel used in 
// the Kalibr toolbox (http://github.com/ethz-asl/kalibr).
// This models only the stochastic component of the sensor. Deterministic
// errors, such as scale and turn - on bias, should be applied to the
// sensordata input prior to using this function.The following paper is used
// as a reference for this function:
//  Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and 
//  Spatial Calibration for Multi-Sensor Systems. In Proceedings of the 
//  IEEE/RSJ International Conference on Intelligent Robots and Systems 
//  (IROS), Tokyo, Japan.
void SensorModel::SensorStocModel_Kalibr(void)
{
	/*--------------- Initializations ---------------*/
	double sensor_noise;
	double arw;
	double sig_arw;
	double dt;
	double a_d;
	double b_d;
	double sig_eta;
	double eta;

	// Period of sensor measurements
	dt = 1.0 / freq;
	/*------------- End Initializations -------------*/

	/*--------------- Rate Random Walk --------------*/
	rrw = rrw + sqrt(dt)*sig_w*randn(0.0, 1.0);
	/*------------- End Rate Random Walk ------------*/

	/*-------------- Angle Random Walk --------------*/
	// Compute the standard deviation of the angle random walk
	sig_arw = sig_meas/sqrt(dt);
	// Find angle random walk contribution
	arw = sig_arw*randn(0.0, 1.0);
	/*------------ End Angle Random Walk ------------*/

	/*----------- Compute Sensor Output -------------*/
	// Compute Total Sensor Noise Contribution
	sensor_noise = arw + rrw;
	// Compute Total Sensor Measurement
	SensorOutput = DetSensorOutput + sensor_noise;
	/*--------- End Compute Sensor Output -----------*/
}
/*--------------- End SensorStocModel_Kalibr -------------*/

/*------------------- SensorModelOutput ------------------*/
// Outputs total corrupted sensor measurement. First deterministic errors are
// applied. Subsequently, stochastic errors are applied. This is a public 
// function in the SensorModel class. The output of the deterministic model 
// is the private variable, DetSensorOutput, which is used as the sensor 
// measurement in the stochastic model onto which noise is added. The output 
// of the stochastic model then is the output of this function and is the 
// public variable, SensorOutput.
void SensorModel::SensorModelOutput(void)
{
	// Corrupt sensor measurement with deterministic errors
	SensorModel::SensorDetModel();

	// Corrupt sensor measurement with stochastic errors
	if (Choice == 1)
	{
		SensorModel::SensorStocModel();
	}
	else
	{
		SensorModel::SensorStocModel_Kalibr();
	}
}
/*----------------- End SensorModelOutput ----------------*/

/*-------------------- Set value of B --------------------*/
// Set the mean square value of bias instability. 
void SensorModel::SetB(double B_input)
{
	B = B_input;
}
/*------------------ End Set value of B ------------------*/

/*-------------------- Set value of K --------------------*/
// Set the mean square value of rate random walk 
void SensorModel::SetK(double K_input)
{
	K = K_input;
}
/*------------------ End Set value of K ------------------*/

/*-------------------- Set value of T --------------------*/
// Set the time window at minimum of Allan variance 
void SensorModel::SetT(double T_input)
{
	T = T_input;
}
/*------------------ End Set value of T ------------------*/

/*------------------- Set value of freq ------------------*/
// Set the frequency of sensor measurements 
void SensorModel::Setfreq(double freq_input)
{
	freq = freq_input;
}
/*----------------- End Set value of freq ----------------*/

/*----------------- Set value of sig_meas ----------------*/
// Set the standard deviation of total sensor measurement (assuming error of mean zero)
void SensorModel::Setsig_meas(double sig_meas_input)
{
	sig_meas = sig_meas_input;
}
/*--------------- End Set value of sig_meas --------------*/

/*------------------ Set value of sig_w ------------------*/
// Set the standard deviation used in rate random walk contribution so that
// low frequency of RRW PSD matches low frequency of the PSD from dataset of sensor data
void SensorModel::Setsig_w(double sig_w_input)
{
	sig_w = sig_w_input;
}
/*---------------- End Set value of sig_w ----------------*/

/*------------------- Set value of bias ------------------*/
// Set the current value of bias instability contribution
void SensorModel::Setbias(double bias_input)
{
	bias = bias_input;
}
/*---------------- End Set value of bias -----------------*/

/*------------------- Set value of rrw -------------------*/
// Set the current value of rate random walk contribution
void SensorModel::Setrrw(double rrw_input)
{
	rrw = rrw_input;
}
/*----------------- End Set value of rrw -----------------*/

/*-------------------- Set value of S --------------------*/
// Set the scale factor error. 
void SensorModel::SetS(double S_input)
{
	S = S_input;
}
/*------------------ End Set value of S ------------------*/

/*-------------------- Set value of Bf -------------------*/
// Set the measurement bias
void SensorModel::SetBf(double Bf_input)
{
	Bf = Bf_input;
}
/*------------------ End Set value of Bf -----------------*/

/*--------- Set value of true sensor measurement ---------*/
// Set the true sensor value that will be corrupted by stochastic errors and deterministic errors
void SensorModel::SetSensorMeasTrue(double SensorMeasTrue_input)
{
	SensorMeasTrue = SensorMeasTrue_input;
}
/*------- End Set value of true sensor measurement -------*/

/*------------------- Get value of freq ------------------*/
// Get the frequency of sensor measurements 
double SensorModel::Getfreq(void)
{
	return freq;
}
/*----------------- End Get value of freq ----------------*/

/*------------- Get Corrupted Sensor Output --------------*/
// Get the corrupted sensor output value
double SensorModel::GetSensorOutput(void)
{
	return SensorOutput;
}
/*----------- End Get Corrupted Sensor Output ------------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Helpers ----------------------------------*/
/*-----------------------------------------------------------------------------*/