package org.firstinspires.ftc.teamcode.blackbox;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.internal.webserver.MimeTypesUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class BlackboxWebServer extends NanoHTTPD {
    @SuppressLint("SdCardPath")
    public static final String UI_PATH = "/sdcard/blackbox/ui";

    public BlackboxWebServer() throws IOException {
        super(4174);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
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
            return newChunkedResponse(Response.Status.OK, MimeTypesUtil.determineMimeType(filePath), inputStream);
        } catch (FileNotFoundException e) {
            return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/html", "File not found.");
        }
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();

        if (uri.startsWith("/sessions")) {
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
