package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public PIDCoefficients coefficients;


    public boolean enableAntiWindup;
    public double saturationMagnitude;

    public double errorSum;

    public double lastDError;

    private ElapsedTime _runtime;
    private double _lastError;
    private double _lastTime;
    private boolean _integrationDisabled;

    public PIDController(PIDCoefficients _coefficients, boolean _enableAntiWindup, double _saturationMagnitude) {
        coefficients = _coefficients;
        enableAntiWindup = _enableAntiWindup;
        saturationMagnitude = _saturationMagnitude;

        _runtime = new ElapsedTime();

        reset();
    }

    public boolean isIntegrationDisabled() {
        return _integrationDisabled;
    }

    public void reset() {
        errorSum = 0;
        _lastError = 0;
        _lastTime = 0;
        _integrationDisabled = false;
        _runtime.reset();
    }

    public double step(double current, double target) {
        double deltaTime = (_runtime.milliseconds() - _lastTime);

        double error = target - current;
        if (!_integrationDisabled) {
            errorSum += (error * deltaTime);
        }
        double dError = (error - _lastError) / deltaTime;

        double output = coefficients.p * error + coefficients.i * errorSum + coefficients.d * dError;

        lastDError = dError;
        _lastError = error;
        _lastTime = _runtime.milliseconds();

        if (enableAntiWindup) {
            _integrationDisabled = (Math.abs(output) >= saturationMagnitude);
        }

        if (Math.abs(output) > saturationMagnitude) {
            // clamp output
            output = Math.signum(output) * saturationMagnitude;
        }

        return output;
    }
}
