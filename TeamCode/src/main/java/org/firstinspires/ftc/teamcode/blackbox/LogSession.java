package org.firstinspires.ftc.teamcode.blackbox;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.OptionsManager;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

public class LogSession implements AutoCloseable {
    @SuppressLint("SdCardPath")
    public static final String BASE_PATH = "/sdcard/blackbox/sessions";

    private final File _basePath = new File(BASE_PATH);
    private File _sessionPath;

    private int _id;

    private MatchPhase _phase;
    private MatchInfo _info;
    private long _matchStart;
    private HashMap<String, String> _options;

    private LogContext _root;

    private ArrayList<LogContext> _contexts;

    private LinearOpMode _opMode;
    private boolean _enabled;

    public LogSession(LinearOpMode opMode, MatchPhase phase, MatchType type, String label) throws IOException {
        _contexts = new ArrayList<LogContext>();
        _options = OptionsManager.getDisplayList();

        _enabled = OptionsManager.getBooleanSetting("logData");

        if (!_basePath.exists()) {
            _basePath.mkdirs();
        }

        _id = getNextSessionID();

        _sessionPath = new File(_basePath, "/" + Integer.toString(_id) + "/");
        _sessionPath.mkdirs();

        _phase = phase;
        _info = new MatchInfo(type, label);
        _root = new LogContext(this, 0, "Opmode", _enabled);
        _opMode = opMode;
        _contexts.add(_root);
    }

    public int getNextSessionID() throws IOException {
        File lastIDFile = new File(_basePath, "last_id.txt");
        int newSessionID = 0;

        if (lastIDFile.exists()) {
            BufferedReader lastIDReader = new BufferedReader(new FileReader(lastIDFile));
            int lastSessionID = Integer.parseInt(lastIDReader.readLine().trim());
            lastIDReader.close();
            newSessionID = lastSessionID + 1;
        }

        saveLastSessionID(lastIDFile, newSessionID);
        return newSessionID;
    }

    private void saveLastSessionID(File lastIDFile, int lastSessionID) throws FileNotFoundException {
        PrintWriter nextIDOut = new PrintWriter(new FileOutputStream(lastIDFile, false));
        nextIDOut.print(lastSessionID);
        nextIDOut.close();
    }

    public File getSessionPath() {
        return _sessionPath;
    }

    public boolean isStopRequested() {
        return _opMode.isStopRequested();
    }

    @Override
    public void close() throws IOException, JSONException {
        _root.close();

        File saveFile = new File(_sessionPath, "session.json");

        JSONObject session = serializeToJSON();
        PrintWriter writer = new PrintWriter(new FileOutputStream(saveFile, false));
        writer.print(session.toString());
        writer.close();
    }

    public JSONObject serializeToJSON() throws JSONException {
        JSONObject sessionJSON = new JSONObject();

        sessionJSON.put("phase", _phase);
        sessionJSON.put("type", _info.type);
        sessionJSON.put("label", _info.label);
        sessionJSON.put("matchStart", _matchStart);

        JSONObject optionsJSON = new JSONObject();
        for (HashMap.Entry<String, String> option : _options.entrySet()) {
            optionsJSON.put(option.getKey(), option.getValue());
        }
        sessionJSON.put("options", optionsJSON);

        JSONArray contextsJSON = new JSONArray();
        for (LogContext context : _contexts) {
            contextsJSON.put(context.serializeToJSON());
        }
        sessionJSON.put("contexts", contextsJSON);

        return sessionJSON;
    }

    public void attachDatastreamable(Datastreamable datastreamable) {
        if (_enabled) {
            _root.attachDatastreamable(datastreamable);
        }
    }

    public void setFact(String key, Object value) {
        _root.setFact(key, value);
    }

    public LogContext createContext(String contextName) {
        LogContext context = new LogContext(this, _contexts.size(), contextName, _enabled);
        _contexts.add(context);
        return context;
    }

    public void startMatch() {
        _matchStart = System.currentTimeMillis();
    }
}
