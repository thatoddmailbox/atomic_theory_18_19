package org.firstinspires.ftc.teamcode.blackbox.sensors;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.AnnotatedYuvRgbFrame;
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;
import org.firstinspires.ftc.teamcode.blackbox.Datastream;
import org.firstinspires.ftc.teamcode.utils.RecognitionResults;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public class WrappedTFObjectDetector extends WrappedSensor<TFObjectDetector> {
    private Method _tfodGetUpdatedAnnotatedFrame;
    private Method _frameGetCopiedBitmap;

    private Datastream<RecognitionResults> _frameStream;

    public WrappedTFObjectDetector(TFObjectDetector sensor, String name) throws InterruptedException {
        super(sensor, name);

        _frameStream = new Datastream<RecognitionResults>("frames");
        _frameStream.enableAttachedImages();

        try {
            _tfodGetUpdatedAnnotatedFrame = TFObjectDetectorImpl.class.getDeclaredMethod("getUpdatedAnnotatedFrame");
            _tfodGetUpdatedAnnotatedFrame.setAccessible(true);

            Field frame = AnnotatedYuvRgbFrame.class.getDeclaredField("frame");
            Class yuvRgbFrameClass = frame.getType();

            _frameGetCopiedBitmap = yuvRgbFrameClass.getDeclaredMethod("getCopiedBitmap");
            _frameGetCopiedBitmap.setAccessible(true);
        } catch (NoSuchMethodException | NoSuchFieldException e) {
            e.printStackTrace();
        }
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                _frameStream
        };
    }

    public void loadModelFromAsset(String tfodModelFile, String tfodLabelGold, String tfodLabelSilver) {
        _sensor.loadModelFromAsset(tfodModelFile, tfodLabelGold, tfodLabelSilver);
    }

    public void activate() {
        _sensor.activate();
    }

    public void shutdown() {
        _sensor.shutdown();
    }

    public List<Recognition> getUpdatedRecognitions() {
        try {
            AnnotatedYuvRgbFrame frameAnnotated = (AnnotatedYuvRgbFrame) _tfodGetUpdatedAnnotatedFrame.invoke(_sensor);
            if (frameAnnotated == null) {
                return null;
            }

            ArrayList<Recognition> recognitionList = new ArrayList<Recognition>(frameAnnotated.getRecognitions());

            Bitmap frameBitmap = (Bitmap) _frameGetCopiedBitmap.invoke(frameAnnotated.getFrame());

            _frameStream.storeReading(new RecognitionResults(recognitionList), frameBitmap);

            return recognitionList;
        } catch (IllegalAccessException | InvocationTargetException e) {
            e.printStackTrace();
        }

        return null;
    }
}
