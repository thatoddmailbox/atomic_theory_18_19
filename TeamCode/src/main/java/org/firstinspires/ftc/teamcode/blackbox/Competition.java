package org.firstinspires.ftc.teamcode.blackbox;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;

public class Competition {
    public String name;
    public ArrayList<Integer> sessions;

    public Competition() {
        name = "";
        sessions = new ArrayList<Integer>();
    }

    public Competition(JSONObject object) throws JSONException {
        name = object.getString("name");
        sessions = new ArrayList<Integer>();

        JSONArray sessionsJSON = object.getJSONArray("sessions");
        for (int i = 0; i < sessionsJSON.length(); i++) {
            sessions.add(sessionsJSON.getInt(i));
        }
    }

    public JSONObject serializeToJSON() throws JSONException {
        JSONObject competitionJSON = new JSONObject();
        competitionJSON.put("name", name);

        JSONArray sessionsJSON = new JSONArray();
        for (Integer sessionID : sessions) {
            sessionsJSON.put(sessionID);
        }
        competitionJSON.put("sessions", sessionsJSON);

        return competitionJSON;
    }
}
