#ifndef PID_H
#define PID_H

class PID {
public:
	/*
	 * Errors
	 */
	double p_error;
	double i_error;
	double d_error;

	/*
	 * Coefficients
	 */
	double Kp;
	double Ki;
	double Kd;

	// No. of updates received
	// Used in twiddle conditionality
	int update_count_;

	// current Value of total Error
	double total_error_;

	// current value of best error
	double best_error_;

	// Deltas for coefficients
	double dp_p_, dp_i_, dp_d_;

	// Count Threshold
	// Useful for error calculation and twiddling
	int count_threshold_;

	// Variable to determine whether Kp, Ki or Kd are adjusted in a twiddle cycle
	int coefficient_choice_;

	// Variable determining whether to increment or decrement deltas in twiddle
	bool increment_;

	/*
	 * Constructor
	 */
	PID();

	/*
	 * Destructor.
	 */
	virtual ~PID();

	/*
	 * Initialize PID.
	 */
	void Init(double Kp, double Ki, double Kd);

	/*
	 * Update the PID error variables given cross track error.
	 */
	void UpdateError(double cte);

	/*
	 * Calculate the total PID error.
	 */
	double TotalError();
	/*
	 * Twiddle method to tune PID coefficients
	 */
	void twiddle();

	/*
	 * Output Value for PID instantiation
	 */
	double output();
};

#endif /* PID_H */
