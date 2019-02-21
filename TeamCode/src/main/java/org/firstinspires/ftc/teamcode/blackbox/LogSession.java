package org.firstinspires.ftc.teamcode.blackbox;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;

public class LogSession implements AutoCloseable {
    private MatchPhase _phase;
    private MatchType _type;
    private String _label;

    private LogContext _root;

    private ArrayList<LogContext> _contexts;

    public LogSession(MatchPhase phase, MatchType type, String label) {
        _contexts = new ArrayList<LogContext>();

        _phase = phase;
        _type = type;
        _label = label;
        _root = new LogContext("Opmode");
        _contexts.add(_root);
    }

    @Override
    public void close() throws Exception {
        _root.close();
    }

    public JSONObject serializeToJSON() throws JSONException {
        JSONObject sessionJSON = new JSONObject();

        sessionJSON.put("phase", _phase);
        sessionJSON.put("type", _type);
        sessionJSON.put("label", _label);

        JSONArray contextsJSON = new JSONArray();
        for (LogContext context : _contexts) {
            contextsJSON.put(context.serializeToJSON());
        }
        sessionJSON.put("contexts", contextsJSON);

        return sessionJSON;
    }

    public void attachDatastreamable(Datastreamable datastreamable) {
        _root.attachDatastreamable(datastreamable);
    }

    public LogContext createContext(String contextName) {
        LogContext context = new LogContext(contextName);
        _contexts.add(context);
        return context;
    }
}
