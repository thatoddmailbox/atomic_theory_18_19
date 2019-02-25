package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;

public class RecognitionResults {
    private ArrayList<Recognition> _recognitionList;

    public RecognitionResults(ArrayList<Recognition> recognitionList) {
        _recognitionList = recognitionList;
    }

    @Override
    public String toString() {
        try {
            JSONArray recognitionsJSON = new JSONArray();
            for (Recognition recognition : _recognitionList) {
                JSONObject recognitionJSON = new JSONObject();

                recognitionJSON.put("label", recognition.getLabel());
                recognitionJSON.put("confidence", recognition.getConfidence());
                recognitionJSON.put("top", recognition.getTop());
                recognitionJSON.put("left", recognition.getLeft());
                recognitionJSON.put("bottom", recognition.getBottom());
                recognitionJSON.put("right", recognition.getRight());

                recognitionsJSON.put(recognitionJSON);
            }
            return recognitionsJSON.toString();
        } catch (JSONException e) {
            e.printStackTrace();
        }

        return null;
    }
}
