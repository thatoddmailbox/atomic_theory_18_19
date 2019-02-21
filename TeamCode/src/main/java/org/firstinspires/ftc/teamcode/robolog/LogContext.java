package org.firstinspires.ftc.teamcode.robolog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.HashMap;

public class LogContext implements AutoCloseable {
    public ArrayList<Datastreamable> datastreamables;
    public HashMap<String, Object> facts;
    public long startTime;
    public long endTime;

    private String _name;
    private boolean _closed;

    public LogContext(String name) {
        datastreamables = new ArrayList<Datastreamable>();
        facts = new HashMap<String, Object>();

        startTime = System.currentTimeMillis();
        endTime = 0;

        _name = name;
        _closed = false;
    }

    public void attachDatastreamable(Datastreamable datastreamable) {
        if (_closed) {
            throw new RuntimeException("Tried to attachDatastreamable() to a closed LogContext!");
        }
        datastreamables.add(datastreamable);
    }

    public JSONObject serializeToJSON() throws JSONException {
        JSONObject contextJSON = new JSONObject();

        contextJSON.put("name", _name);
        contextJSON.put("start", startTime);
        contextJSON.put("end", endTime);

        JSONObject factsJSON = new JSONObject();
        for (HashMap.Entry<String, Object> fact : facts.entrySet()) {
            factsJSON.put(fact.getKey(), fact.getValue());
        }
        contextJSON.put("facts", factsJSON);

        JSONArray datastreamablesJSON = new JSONArray();
        for (Datastreamable datastreamable : datastreamables) {
            JSONObject datastreamableJSON = new JSONObject();

            datastreamableJSON.put("name", datastreamable.getName());

            JSONArray datastreamsJSON = new JSONArray();
            for (Datastream datastream : datastreamable.getDatastreams()) {
                datastreamsJSON.put(datastream.serializeToJSON());
            }
            datastreamableJSON.put("datastreams", datastreamsJSON);

            datastreamablesJSON.put(datastreamableJSON);
        }
        contextJSON.put("datastreamables", datastreamablesJSON);

        return contextJSON;
    }

    @Override
    public void close() throws Exception {
        endTime = System.currentTimeMillis();
        _closed = true;
    }
}
