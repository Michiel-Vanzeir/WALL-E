#ifndef PID_H
#define PID_H

class PIDController {
    public:
      PIDController(double proportional_gain, double derivative_gain, double integral_gain);

      double calculate(double error);
    private:
        void update(double error);

        double Kp = 0;
        double Kd = 0;
        double Ki = 0;

        double error_sum = 0;
        double previous_error = 0;
};

#endif // PID_H