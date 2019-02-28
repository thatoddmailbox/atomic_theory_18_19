package org.firstinspires.ftc.teamcode.blackbox;

import org.json.JSONException;
import org.json.JSONObject;

public class MatchInfo {
    public MatchType type;
    public String label;

    public MatchInfo() {
        type = MatchType.OTHER;
        label = "";
    }

    public MatchInfo(MatchType typeIn, String labelIn) {
        type = typeIn;
        label = labelIn;
    }

    public MatchInfo(JSONObject infoJSON) throws JSONException {
        type = MatchType.valueOf(infoJSON.getString("type"));
        label = infoJSON.getString("label");
    }

    public JSONObject serializeToJSON() throws JSONException {
        JSONObject infoJSON = new JSONObject();

        infoJSON.put("type", type);
        infoJSON.put("label", label);

        return infoJSON;
    }
}
