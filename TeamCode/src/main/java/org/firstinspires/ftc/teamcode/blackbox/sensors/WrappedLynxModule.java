package org.firstinspires.ftc.teamcode.blackbox.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import org.firstinspires.ftc.teamcode.blackbox.Datastream;

public class WrappedLynxModule extends WrappedSensor<LynxModule> {
    private Datastream<Double> _twelveVoltStream;
    private Datastream<Double> _fiveVoltStream;
    private Datastream<Double> _batteryCurrentStream;
    private Datastream<Double> _motor0Stream;
    private Datastream<Double> _motor1Stream;
    private Datastream<Double> _motor2Stream;
    private Datastream<Double> _motor3Stream;

    private long lastDatastreamUpdate = 0;

    public WrappedLynxModule(LynxModule sensor, String name) throws InterruptedException {
        super(sensor, name);

        _twelveVoltStream = new Datastream<Double>("+12V rail");
        _fiveVoltStream = new Datastream<Double>("+5V rail");
        _batteryCurrentStream = new Datastream<Double>("battery current");
        _motor0Stream = new Datastream<Double>("motor 0 current");
        _motor1Stream = new Datastream<Double>("motor 1 current");
        _motor2Stream = new Datastream<Double>("motor 2 current");
        _motor3Stream = new Datastream<Double>("motor 3 current");
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                _twelveVoltStream,
                _fiveVoltStream,
                _batteryCurrentStream,
                _motor0Stream,
                _motor1Stream,
                _motor2Stream,
                _motor3Stream
        };
    }

    public void pumpDatastream() {
        long currentTime = System.currentTimeMillis();
        long difference = currentTime - lastDatastreamUpdate;
        if (difference > 10) {
            _twelveVoltStream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.BATTERY_MONITOR));
            _fiveVoltStream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.FIVE_VOLT_MONITOR));
            _batteryCurrentStream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.BATTERY_CURRENT));
            _motor0Stream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.MOTOR0_CURRENT));
            _motor1Stream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.MOTOR1_CURRENT));
            _motor2Stream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.MOTOR2_CURRENT));
            _motor3Stream.storeReading(readADCFromHub(LynxGetADCCommand.Channel.MOTOR3_CURRENT));
            lastDatastreamUpdate = System.currentTimeMillis();
        }
    }

    public LynxModule getRawHub() {
        return _sensor;
    }

    /*
     * communication functions
     */
    public LynxGetBulkInputDataResponse getBulkData() {
        LynxGetBulkInputDataCommand bulkInputDataCommand = new LynxGetBulkInputDataCommand(_sensor);
        try {
            return bulkInputDataCommand.sendReceive();
        } catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
        return null;
    }

    public double readADCFromHub(LynxGetADCCommand.Channel channel) {
        LynxGetADCCommand command = new LynxGetADCCommand(_sensor, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            return response.getValue();
        } catch (InterruptedException | RuntimeException | LynxNackException e) {
            e.printStackTrace();
        }
        return Double.MAX_VALUE;
    }
}
