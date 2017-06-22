#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

/*
 * TODO: Complete the PID class.
 */

#define DEBUG 0
PID::PID() {}

PID::~PID() {}

// NOTE: Changed variable names to avoid any chance of incorrect scoping
void PID::Init(double lKp, double lKi, double lKd)
{
	// Initialize Gains
	// This could also be done as a copy constructor but effectively makes no difference

	Kp = lKp;
	Kd = lKd;
	Ki = lKi;

	//Initialize Errors
	p_error = 0.00;
	i_error = 0.00;
	d_error = 0.00;

	// Initialize update counter
	update_count_ = 0;

	// Initialize Errors
	total_error_ = 0.00;
	best_error_ = 9999.0;

	// Initialize counter
	count_threshold_ = 0;

	// Initialize deltas
	dp_p_ = Kp/10.0;
	dp_i_ = Ki/10.0;
	dp_d_ = Kd/10.0;

	// Initialize Coefficient choice to zero (Means Kp)
	coefficient_choice_ = 0;

	// Try increment first
	increment_ = true;

}

void PID::UpdateError(double cte)
{
	// Update the differential error
	d_error = cte - p_error;

	// update the p_error
	p_error = cte;

	// update the Integral Error
	i_error += cte;

	// increment update counter
	// Use RMS for total Error
	if(update_count_>count_threshold_)
		total_error_ += cte*cte ;
	update_count_++;

	// print for debugging
	if (DEBUG)
		printf("\n Current Values of errors are P_Error : %.02f I_Error: %.02f D_Error: %.02f", p_error, i_error, d_error);

}

double PID::TotalError()
{
	//	// calculate the total error
	//	total_error_ = Kp*p_error + Ki*i_error + Kd*d_error;
	//
	//	// print for debugging
	//	if (DEBUG)
	//		printf("\n Total Error is %.02f", total_error_);
	//
	//	// return value
	//	return total_error_;

	return total_error_/(update_count_ - count_threshold_);
}

double PID::output()
{
	// Calculate output
	double output_val = -1.0 * (Kp*p_error + Kd*d_error + Ki*i_error);

	// Limit between zero and one
	if (output_val > 1.0)
		output_val = 1.0;
	else if (output_val < -1.0)
		output_val = -1.0;

	if(DEBUG)
		printf("\n Output is %.03f", output_val);

	// Return output
	return output_val;
}


void PID::twiddle()
{

	//get Current error
	double current_error = TotalError();

	// copy Update count to local variable
	// Useful if Twiddle is later abstracted away into a parallel thread,
	// Will need mutexes
	int local_update_count = update_count_;

	// reset total error to zero
	total_error_ = 0;
	// reset number of steps to zero because twiddle will need to start again
	update_count_ = 0;


	// If current best error is too large
	if(best_error_ > 999)
	{
		best_error_ = current_error;

		if (DEBUG)
			printf("\n Best error too big \n Best Param Set (Kp,Ki,Kd) is ( %.03f,%.03f,%.03f)", Kp, Ki, Kd);
		// Increment Kp
		Kp += dp_p_;

		// return
		return;
	}

	// Reinitialize individual errors
	i_error = 0;
	d_error = 0;
	p_error = 0;

	// If the current error is less than best error
	if(current_error < best_error_ && local_update_count > 2*count_threshold_)
	{
		// Set best error to current value
		best_error_ = current_error;

		// Adjust one of the three coefficients
		if(coefficient_choice_ == 0)
			dp_p_ *= 1.1;
		else if(coefficient_choice_ == 1)
			dp_i_ *= 1.1;
		else
			dp_d_ *= 1.1;

		// Cyclically adjust coefficient choice
		coefficient_choice_ = (coefficient_choice_+1)%3;

		// Since We incremented , set increment to True
		increment_ = true;

		// Print for Debugging
		if (DEBUG)
			printf("\n Best Param Set (Kp,Ki,Kd) is ( %.03f,%.03f,%.03f)", Kp, Ki, Kd);

	}
	else
	{
		// Since Last update was an increment and current error was worse
		// than best error, set increment to false
		if(increment_ == true)
			increment_ = false;
		else
		{
			// Since we are decrementing now
			// Adjust Values accordingly based on current coefficient choice
			if (coefficient_choice_ == 0) {
				Kp += dp_p_;
				dp_p_ *= 0.9;
			}
			else if (coefficient_choice_ == 1) {
				Ki += dp_i_;
				dp_i_ *= .9;
			}
			else {
				Kd += dp_d_;
				dp_d_ *= .9;
			}

			// Cyclically adjust coefficient choice
			coefficient_choice_ = (coefficient_choice_ + 1) % 3;

			// Since increment was false, Set it to true so next attempt is upwards
			increment_ = true;
		}
	}
	// In any case, Readjust Coefficients after conditional check based on coefficient choice
	if(coefficient_choice_ == 0)
	{
		if(increment_) Kp += dp_p_;
		else Kp-= 2*dp_p_;
	}
	else if(coefficient_choice_ == 1)
	{
		if(increment_) Ki += dp_i_;
		else Ki -= 2*dp_i_;
	}
	else if(coefficient_choice_ == 2)
	{
		if(increment_) Kd += dp_d_;
		else Kd-= 2*dp_d_;
	}
}

