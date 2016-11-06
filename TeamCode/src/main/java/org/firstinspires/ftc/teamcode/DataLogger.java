package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * Created by Olavi Kamppari on 9/9/2015.
 * <p>
 * Added to Github on 11/16/2015 (https://github.com/OliviliK/FTC_Library/blob/master/DataLogger.java)
 * <p>
 * modified by jjkd 04-Nov-2016
 */

public class DataLogger {
    private Writer writer;
    private StringBuffer lineBuffer;
    private long msBase;
    private long nsBase;

    public DataLogger(String dirName, String fileName) {

        // return;

        //    telemetry.addData("00:DataLogger","Invoked...");

        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            //        telemetry.addData("00:DataLogger","ExternalStorage is writeable");
        } else {
            //        telemetry.addData("00:DataLogger","ExternalStorage is NOT writeable");
        }

        File logdir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), dirName);
        File logfile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), dirName + "/" + fileName + ".csv");

        if (!logdir.mkdirs()) {
            //       telemetry.addData("00:DataLogger", "Directory not created");
        } else {
            //       telemetry.addData("00:DataLogger", "Directory is created");
        }

        // Make sure that the directory exists

        try {
            writer = new FileWriter(logfile);
            lineBuffer = new StringBuffer(128);
        } catch (IOException e) {
        }

        msBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
        addField("sec");
//        addField("d ms");
    }

    private void flushLineBuffer() {
        long milliTime, nanoTime;

        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            writer.flush();
            lineBuffer.setLength(0);
        } catch (IOException e) {
        }
        milliTime = System.currentTimeMillis();
        nanoTime = System.nanoTime();
        addField(String.format("%.3f", (milliTime - msBase) / 1.0E3));
        addField(String.format("%.3f", (nanoTime - nsBase) / 1.0E6));
        nsBase = nanoTime;
    }

    public void closeDataLogger() {
        try {
            writer.close();
        } catch (IOException e) {
        }
    }

    public void addField(String s) {
        if (lineBuffer.length() > 0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
    }

    public void addField(char c) {
        if (lineBuffer.length() > 0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(c);
    }

    public void addField(boolean b) {
        addField(b ? '1' : '0');
    }

    public void addField(byte b) {
        addField(Byte.toString(b));
    }

    public void addField(short s) {
        addField(Short.toString(s));
    }

    public void addField(long l) {
        addField(Long.toString(l));
    }

    public void addField(float f) {
        addField(Float.toString(f));
    }

    public void addField(double d) {
        addField(Double.toString(d));
    }

    public void newLine() {
        flushLineBuffer();
    }

    @Override
    protected void finalize() throws Throwable {
        closeDataLogger();
        super.finalize();
    }
}