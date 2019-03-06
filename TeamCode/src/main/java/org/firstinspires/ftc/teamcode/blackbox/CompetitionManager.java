package org.firstinspires.ftc.teamcode.blackbox;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

public class CompetitionManager {
    public static final String COMPETITIONS_FILE = LogSession.BASE_PATH + "/competitions.json";

    public static void init() {
        File competitionFile = new File(COMPETITIONS_FILE);
        if (!competitionFile.exists()) {
            try {
                competitionFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static ArrayList<Competition> getCompetitions() throws IOException, JSONException {
        ArrayList<Competition> competitions = new ArrayList<Competition>();

        BufferedReader competitionFileReader = new BufferedReader(new FileReader(COMPETITIONS_FILE));
        String competitionFileContents = competitionFileReader.readLine();

        if (competitionFileContents == null || competitionFileContents.isEmpty()) {
            return competitions;
        }

        JSONArray competitionJSONArray = new JSONArray(competitionFileContents);
        for (int i = 0; i < competitionJSONArray.length(); i++) {
            competitions.add(new Competition(competitionJSONArray.getJSONObject(i)));
        }

        return competitions;
    }

    public static void saveCompetitionList(ArrayList<Competition> competitions) throws IOException, JSONException {
        JSONArray competitionsJSON = new JSONArray();

        for (Competition competition : competitions) {
            competitionsJSON.put(competition.serializeToJSON());
        }

        String competitionsJSONString = competitionsJSON.toString();

        PrintWriter writer = new PrintWriter(COMPETITIONS_FILE);
        writer.print(competitionsJSONString);
        writer.flush();
        writer.close();
    }

    public static void addSessionKeyToCompetition(int competitionIndex, int sessionKey) throws IOException, JSONException {
        ArrayList<Competition> competitions = getCompetitions();
        Competition competition = competitions.get(competitionIndex);
        competition.sessions.add(sessionKey);
        competitions.set(competitionIndex, competition);
        saveCompetitionList(competitions);
    }
}
