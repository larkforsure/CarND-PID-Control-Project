#include "PID.h"
#include <iostream>

/*
* TODO: Complete the PID class.
*/

PID::PID(std::string name) :
	name(name), p_error(0.), i_error(0.), d_error(0.), Kp(0.), Ki(0.), Kd(0.)
	, steps(0ull), startup_steps(300), error(0.), best_error(9e15)
	, current_index(0), current_state(TWIDDLE_START) 
{
	p.resize(3, 0.);
	dp.resize(3, 0.);
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	/*
	* Coefficients
	*/
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	// use paramter/10 as initial probe steps
	dp[0] = Kp / 10;
	dp[1] = Ki / 10;
	dp[2] = Kd / 10;
}

// Call this when the CTE becomes large again
void PID::Recaliberate() {
	std::cout << "[" << name << "]" << " Re-caliberate ";
	d_error = 0.;
	p_error = 0.;
	i_error = 0.;
	steps = 0;
	// already have some speed, need the fast reflection here
	startup_steps = 0;
	// use paramter/30 as initial probe steps
	dp[0] = Kp / 10.0;
	dp[1] = Ki / 10.0;
	dp[2] = Kd / 10.0;
}

void PID::UpdateError(double cte) {
	d_error = (cte - p_error);
	p_error = cte;
	i_error += cte;

	steps += 1;
	//std::cout << "STEP " << steps << " Error: [" << p_error << ", " << d_error << ", " << i_error << "]" << std::endl;
	if(steps > startup_steps) error += (cte*cte);
}

double PID::TotalError() {
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}



bool PID::UpdateTwiddle() {
	// without startup steps, the car will stall in the start place with dp increasing, error decreasing
	if (steps <= startup_steps)
		return false;
	if (dp[0] + dp[1] + dp[2] < 0.02) {
		std::cout << "[" <<name << "]" << " Achieved optimum: " << "Kp=" << this->Kp << " , Ki=" << this->Ki << " , Kd=" << this->Kd << " , best_error=" << this->best_error << std::endl;
		return true;
	}

	// normailize error with total steps
	double error = (this->error) / (steps- startup_steps);
	std::cout << "[" << name << "]" << " TWIDDLE--- STEP " << steps << " err=" << error << " best_error=" << best_error << std::endl;
	bool next_parameter = false;
	if (current_state == TWIDDLE_START) {
		// firstly probe up
		current_state = TWIDDLE_UP;
		p[current_index] += dp[current_index];
	}
	else if (current_state == TWIDDLE_UP) {
		if (error < best_error) {
			best_error = error;
			std::cout << "[" << name << "]" << " UP succeeded best_error=" << best_error;
			std::cout << " update dp[" << current_index << "] from " << dp[current_index] << " to " << (1.1 * dp[current_index]) << std::endl;
			dp[current_index] *= 1.1;
			next_parameter = true;
			current_state = TWIDDLE_START;
		}
		else {
			// change to probe down
			std::cout << "[" << name << "]" << " UP , update p[" << current_index << "] from " << p[current_index] << " to " << (p[current_index] - 2 * dp[current_index]) << std::endl;
			p[current_index] -= 2 * dp[current_index];
			current_state = TWIDDLE_DOWN;
		}
	}
	else if (current_state == TWIDDLE_DOWN) {
		if (error < best_error) {
			best_error = error;
			std::cout << "[" << name << "]" << " DOWN succeeded best_error=" << best_error;
			std::cout << " update dp[" << current_index << "] from " << dp[current_index] << " to " << (1.1 * dp[current_index]) << std::endl;
			dp[current_index] *= 1.1;
		}
		else {
			std::cout << "[" << name << "]" << " DOWN , update p[" << current_index << "] from " << p[current_index] << " to " << (p[current_index] + dp[current_index]);
			std::cout << " update dp[" << current_index << "] from " << dp[current_index] << " to " << (0.9 * dp[current_index]) << std::endl;
			// both up & down failed, dp too large. 
			// back to the orignal position firstly
			p[current_index] += dp[current_index];
			// decrease dp
			dp[current_index] *= 0.9;
		}
		next_parameter = true;
		current_state = TWIDDLE_START;
	}

	// move to the next paramter
	if (next_parameter) {
		current_index = (current_index + 1) % 3;
	}
	
	this->Kp = p[0];
	this->Ki = p[1];
	this->Kd = p[2];

	return false;
}