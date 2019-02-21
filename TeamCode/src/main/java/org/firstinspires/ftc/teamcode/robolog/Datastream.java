package org.firstinspires.ftc.teamcode.robolog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;

public class Datastream<T> {
    public String name;
    public ArrayList<TimestampedReading<T>> data;

    public Datastream(String nameIn) {
        name = nameIn;
    }

    public void clear() {
        data.clear();
    }

    public void storeReading(T value) {
        data.add(new TimestampedReading<T>(System.currentTimeMillis(), value));
    }

    public JSONObject serializeToJSON() throws JSONException {
        // TODO: make more efficient? less tiny JSONObject allocations, more directly streaming data out

        JSONObject datastreamJSON = new JSONObject();
        datastreamJSON.put("name", name);

        JSONArray dataJSON = new JSONArray();
        for (TimestampedReading<T> reading : data) {
            JSONObject readingJSON = new JSONObject();
            readingJSON.put("t", reading.time);
            readingJSON.put("v", reading.value);
            dataJSON.put(readingJSON);
        }
        datastreamJSON.put("data", dataJSON);

        return datastreamJSON;
    }
}
