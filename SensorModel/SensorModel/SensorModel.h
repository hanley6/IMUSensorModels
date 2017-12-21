class SensorModel
{
public:
	// Default Constructor
	SensorModel();

	// Parameterized Constructor
	// Choose anything besides 1 for using Kalibr
	SensorModel(int Kalibr);

	/*----- Set Values that describe errors (both stochastic and deterministic of an IMU sensor) ------*/
	// Stochastic Parameters
	void SetB(double B_input);					// Mean Square Value of Bias Instability
	void SetK(double K_input);					// Mean Square Value of Rate Random Walk
	void SetT(double T_input);					// Time Window at Minimum of Allan Variance
	void Setfreq(double freq_input);			// Frequency of Sensor Measurements
	void Setsig_meas(double sig_meas_input);	// Standard Deviation of Total Sensor Measurement (assuming error of mean zero)
	void Setsig_w(double sig_w_input);			// Standard deviation used in rate random walk contribution so that
												// low frequency of RRW PSD matches low frequency of the PSD from dataset of sensor data
	void Setbias(double bias_input);			// Current value of bias instability contribution
	void Setrrw(double rrw_input);				// Current value of rate random walk contribution

	// Deterministic Parameters
	void SetS(double S_input);					// Scale factor error
	void SetBf(double Bf_input);				// Measurement bias
	/*--- End Set Values that describe errors (both stochastic and deterministic of an IMU sensor) ----*/

	/*---------- Get Values that describe IMU measurements ----------*/
	// Get the frequency of sensor measurements
	double Getfreq(void);
	// Get the corrupted sensor output value
	double GetSensorOutput(void);
	/*-------- End Get Values that describe IMU measurements --------*/

	// Set the true sensor value that will be corrupted by stochastic errors and deterministic errors
	void SetSensorMeasTrue(double SensorMeasTrue_input);

	// Compute Corrupted Sensor Output
	void SensorModelOutput(void);
private:
	// Choice flag is 1 if using normal IMU model and anything else if using Kalibr model.
	int Choice;

	// Corrupted Deterministic Sensor Output
	double DetSensorOutput;

	// True sensor value that will be corrupted by stochastic errors and deterministic errors
	double SensorMeasTrue;

	// Corrupted Sensor Output Value
	double SensorOutput;

	// Function that Computes the Sensor Output with Deterministic Error Contributors
	void SensorDetModel(void);

	// Function that Computes the Sensor Output with Stochastic Error Contributors
	void SensorStocModel(void);

	// Function that Computes the Sensor Output with Stochastic Error Contributors using Kalibr Model.
	void SensorStocModel_Kalibr(void);

	/*----- Parameters that describe errors (both stochastic and deterministic of an IMU sensor) -----*/
	// Stochastic Parameters
	double B;		 // Mean Square Value of Bias Instability
	double K;		 // Mean Square Value of Rate Random Walk
	double T;		 // Time Window at Minimum of Allan Variance
	double freq;	 // Frequency of Sensor Measurements
	double sig_meas; // Standard Deviation of Total Sensor Measurement (assuming error of mean zero)
	double sig_w;	 // Standard deviation used in rate random walk contribution so that 
					 // low frequency of RRW PSD matches low frequency of the PSD from 
					 // dataset of sensor data
	double bias;	 // Current Value of Bias Instability Contribution
	double rrw;		 // Current Value of Rate Random Walk Contribution

					 // Deterministic Parameters
	double S;		// Scale factor error
	double Bf;		// Measurement bias
	/*--- End Parameters that describe errors (both stochastic and deterministic of an IMU sensor) ---*/
};