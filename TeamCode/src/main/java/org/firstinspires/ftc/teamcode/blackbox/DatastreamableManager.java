package org.firstinspires.ftc.teamcode.blackbox;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

public class DatastreamableManager implements AutoCloseable {
    private Datastreamable _datastreamable;

    private Thread _writerThread;

    public DatastreamableManager(LogContext context, int id, Datastreamable datastreamable) {
        _datastreamable = datastreamable;
        _writerThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    // set up datastreamable path
                    File contextPath = context.getContextPath();
                    File datastreamablePath = new File(contextPath, "/datastreamables/" + Integer.toString(id));
                    datastreamablePath.mkdirs();

                    // set up datastream writers
                    Datastream[] datastreams = _datastreamable.getDatastreams();
                    ArrayList<PrintWriter> datastreamWriters = new ArrayList<PrintWriter>();
                    for (int i = 0; i < datastreams.length; i++) {
                        File datastreamFile = new File(datastreamablePath, Integer.toString(i) + ".csv");
                        datastreamFile.createNewFile();

                        PrintWriter datastreamWriter = new PrintWriter(datastreamFile);
                        datastreamWriter.println("time,value");
                        datastreamWriter.flush();

                        datastreamWriters.add(datastreamWriter);
                    }

                    // write
                    while (!context.getSession().isStopRequested()) {
                        for (int i = 0; i < datastreams.length; i++) {
                            Datastream datastream = datastreams[i];
                            PrintWriter datastreamWriter = datastreamWriters.get(i);

                            TimestampedReading reading = null;
                            while ((reading = (TimestampedReading) datastream.data.poll()) != null) {
                                datastreamWriter.print(reading.time);
                                datastreamWriter.print(",");
                                datastreamWriter.println(reading.value);
                            }
                        }
                    }

                    // save actual data
                    for (PrintWriter writer : datastreamWriters) {
                        writer.flush();
                        writer.close();
                    }

                    // save datastreamable info
                    JSONObject datastreamableInfoJSON = new JSONObject();
                    datastreamableInfoJSON.put("name", _datastreamable.getName());

                    JSONArray datastreamsJSON = new JSONArray();
                    for (Datastream datastream : datastreams) {
                        JSONObject datastreamJSON = new JSONObject();
                        datastreamJSON.put("name", datastream.name);
                        datastreamsJSON.put(datastreamJSON);
                    }

                    datastreamableInfoJSON.put("datastreams", datastreamsJSON);

                    File datastreamableInfoFile = new File(datastreamablePath, "info.json");
                    PrintWriter writer = new PrintWriter(new FileOutputStream(datastreamableInfoFile, false));
                    writer.print(datastreamableInfoJSON.toString());
                    writer.close();
                } catch (IOException | JSONException e) {
                    e.printStackTrace();
                }
            }
        });
        _writerThread.start();
    }

    @Override
    public void close() throws Exception {

    }
}
