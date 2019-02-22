package org.firstinspires.ftc.teamcode.blackbox;

import java.io.File;
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
                    File datastreamablePath = new File(contextPath, "/ds/" + Integer.toString(id));
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

                    for (PrintWriter writer : datastreamWriters) {
                        writer.flush();
                        writer.close();
                    }
                } catch (IOException e) {
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
