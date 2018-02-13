#include "PID.h"
#include <iostream>

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	Kp_derivative = Kp;
	Ki_derivative = Ki;
	Kd_derivative = Kd;
	Kp_derivative_sum = 0;
	Kd_derivative_sum = 0;
	Ki_derivative_sum = 0;
	iteration = 1;
	iter_interval = 100;
	count = 1;
}

void PID::UpdateError(double cte) {

	if (count >= 1000) {
		Kp_derivative = - (cte - p_error);
		Kd_derivative = - (Kp_derivative - d_error);
		Ki_derivative = - cte;
		Kp_derivative_sum += Kp_derivative;
		Kd_derivative_sum += Kd_derivative;
		Ki_derivative_sum += Ki_derivative;
		iteration = iteration + 1;
	}

	// derivative gain
	d_error = cte - p_error;
	// proportional gain
	p_error = cte;
	// integral gain
	i_error += cte;

	count = count + 1;
}

double PID::TotalError() {
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}

// alpha is the learning rate
void PID::GradientDescent(double alpha) {
	if (iteration % iter_interval == 0 && count >= 1000) {
		double Kp_derivative_avg = Kp_derivative_sum/iter_interval;
		double Kd_derivative_avg = Kd_derivative_sum/iter_interval;
		double Ki_derivative_avg = Ki_derivative_sum/iter_interval;
		Kp += alpha*Kp_derivative_avg;
		Kd += alpha*Kd_derivative_avg;
		Ki += alpha*Ki_derivative_avg;
		Kp_derivative_sum = 0;
		Kd_derivative_sum = 0;
		Ki_derivative_sum = 0;
	}
	cout << "Count: " << count << endl;
	cout << "Iteration: " << iteration << endl;
	cout << "Kp: " << Kp << endl;
	cout << "Kd: " << Kd << endl;
	cout << "Ki: " << Ki << endl;
}
