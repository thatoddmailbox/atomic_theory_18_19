package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.Datastream;
import org.firstinspires.ftc.teamcode.blackbox.Datastreamable;

public class PIDController implements Datastreamable {
    public PIDCoefficients coefficients;

    public boolean enableAntiWindup;
    public double saturationMagnitude;

    public double errorSum;

    public double lastDError;

    private ElapsedTime _runtime;
    private double _lastError;
    private double _lastTime;
    private boolean _integrationDisabled;
    private String _label;

    private Datastream<Double> _currentStream;
    private Datastream<Double> _targetStream;
    private Datastream<Double> _outputStream;

    public PIDController(String label, PIDCoefficients _coefficients, boolean _enableAntiWindup, double _saturationMagnitude) {
        coefficients = _coefficients;
        enableAntiWindup = _enableAntiWindup;
        saturationMagnitude = _saturationMagnitude;

        _runtime = new ElapsedTime();
        _label = label;

        _currentStream = new Datastream<Double>("current");
        _targetStream = new Datastream<Double>("target");
        _outputStream = new Datastream<Double>("output");

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

        _currentStream.storeReading(current);
        _targetStream.storeReading(target);
        _outputStream.storeReading(output);

        return output;
    }

    @Override
    public String getName() {
        return "PID - " + _label;
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                _currentStream,
                _targetStream,
                _outputStream
        };
    }
}
