#include "pid.h"

PIDController::PIDController(double proportional_gain, double derivative_gain, double integral_gain)
    : Kp(proportional_gain), Kd(derivative_gain), Ki(integral_gain) {}

double PIDController::calculate(double error) {
    double PID_output = Kp*error + Ki*error_sum + Kd*(error-previous_error);
    
    update(error); // Updates error_sum and previous_error

    return PID_output;

}

void PIDController::update(double error) {
    error_sum += error;
    previous_error = error;
}