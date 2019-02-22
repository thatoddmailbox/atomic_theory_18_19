package org.firstinspires.ftc.teamcode.blackbox;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

public class LogContext implements AutoCloseable {
    public HashMap<String, Object> facts;
    public long startTime;
    public long endTime;

    private ArrayList<Datastreamable> _datastreamables;
    private ArrayList<DatastreamableManager> _datastreamableManagers;

    private String _name;
    private boolean _closed;
    private LogSession _session;
    private File _path;

    public LogContext(LogSession session, int id, String name) {
        facts = new HashMap<String, Object>();
        startTime = System.currentTimeMillis();
        endTime = 0;

        _datastreamables = new ArrayList<Datastreamable>();
        _datastreamableManagers = new ArrayList<DatastreamableManager>();

        _name = name;
        _closed = false;
        _session = session;
        _path = new File(session.getSessionPath(), "/ctx/" + Integer.toString(id));
        _path.mkdirs();
    }

    public void attachDatastreamable(Datastreamable datastreamable) {
        if (_closed) {
            throw new RuntimeException("Tried to attachDatastreamable() to a closed LogContext!");
        }
        _datastreamables.add(datastreamable);
        _datastreamableManagers.add(new DatastreamableManager(this, _datastreamableManagers.size(), datastreamable));
    }

    public String getName() {
        return _name;
    }

    public LogSession getSession() {
        return _session;
    }

    public File getContextPath() {
        return _path;
    }

    public void setFact(String key, Object value) {
        facts.put(key, value);
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

        return contextJSON;
    }

    @Override
    public void close() {
        endTime = System.currentTimeMillis();
        _closed = true;
    }
}