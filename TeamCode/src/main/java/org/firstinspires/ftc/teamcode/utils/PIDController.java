package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public PIDCoefficients coefficients;

    public double errorSum;

    public double lastDError;

    private ElapsedTime _runtime;
    private double _lastError;
    private double _lastTime;

    public PIDController(PIDCoefficients _coefficients) {
        coefficients = _coefficients;

        _runtime = new ElapsedTime();

        reset();
    }

    public void reset() {
        errorSum = 0;
        _lastError = 0;
        _lastTime = 0;
        _runtime.reset();
    }

    public double step(double current, double target) {
        double deltaTime = (_runtime.milliseconds() - _lastTime);

        double error = target - current;
        errorSum += (error * deltaTime);
        double dError = (error - _lastError) / deltaTime;

        double output = coefficients.p * error + coefficients.i * errorSum + coefficients.d * dError;

        lastDError = dError;
        _lastError = error;
        _lastTime = _runtime.milliseconds();

        return output;
    }
}
