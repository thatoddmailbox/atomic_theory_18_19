package org.firstinspires.ftc.teamcode.blackbox;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.internal.webserver.MimeTypesUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class BlackboxWebServer extends NanoHTTPD {
    @SuppressLint("SdCardPath")
    public static final String UI_PATH = "/sdcard/blackbox/ui";

    public BlackboxWebServer() throws IOException {
        super(4174);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
    }

    public String findMimeType(String path) {
        String type = MimeTypesUtil.determineMimeType(path);
        if (type != null) {
            return type;
        }

        if (path.endsWith(".csv")) {
            return "text/csv";
        }

        return "application/octet-stream";
    }

    public Response serveFile(String filePath, String localBasePath) {
        File filePathFile = new File(filePath);
        try {
            if (!filePathFile.getAbsolutePath().equals(filePathFile.getCanonicalPath())) {
                return newFixedLengthResponse("Path traversal?");
            }
        } catch (IOException e) {
            return newFixedLengthResponse("Path traversal?");
        }

        File requestedFile = new File(localBasePath, filePathFile.getAbsolutePath());

        try {
            FileInputStream inputStream = new FileInputStream(requestedFile);
            return newChunkedResponse(Response.Status.OK, findMimeType(filePath), inputStream);
        } catch (FileNotFoundException e) {
            return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/html", "File not found.");
        }
    }

    public Response listAllLogSessions() {
        JSONArray logSessionsJSON = new JSONArray();

        File sessionsFile = new File(LogSession.BASE_PATH);
        String[] folderContents = sessionsFile.list();

        for (String entry : folderContents) {
            File folderFile = new File(sessionsFile, entry);
            if (folderFile.isFile()) {
                continue;
            }

            File sessionFile = new File(folderFile, "session.json");
            try {
                BufferedReader sessionFileReader = new BufferedReader(new FileReader(sessionFile));
                String sessionFileContents = sessionFileReader.readLine();
                if (sessionFileContents == null || sessionFileContents.isEmpty()) {
                    continue;
                }
                JSONObject sessionFileJSON = new JSONObject(sessionFileContents);
                JSONObject logSessionJSON = new JSONObject();

                logSessionJSON.put("path", entry);
                logSessionJSON.put("phase", sessionFileJSON.get("phase"));
                logSessionJSON.put("opmode", sessionFileJSON.getJSONArray("contexts").getJSONObject(0).getJSONObject("facts").getString("Name"));
                logSessionJSON.put("matchType", sessionFileJSON.get("type"));
                logSessionJSON.put("matchLabel", sessionFileJSON.get("label"));
                logSessionJSON.put("matchStart", sessionFileJSON.get("matchStart"));

                logSessionsJSON.put(logSessionJSON);
            } catch (IOException | JSONException e) {
                e.printStackTrace();
            }
        }

        return newFixedLengthResponse(Response.Status.OK, "application/json", logSessionsJSON.toString());
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();

        if (uri.startsWith("/api")) {
            if (uri.equals("/api/listSessions")) {
                return listAllLogSessions();
            }
            return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/html", "File not found.");
        } else if (uri.startsWith("/sessions")) {
            String filePath = uri.substring("/sessions".length());
            return serveFile(filePath, LogSession.BASE_PATH);
        } else {
            // probably a ui-related thing
            if (uri.equals("/")) {
                uri = "/index.html";
            }
            return serveFile(uri.substring(1), UI_PATH);
        }
    }
}
