package frc.robot.filters;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class Record {
    static public String COMMA_DELIMITER = ",";

    public Record(long time, double dist, double rate, double estDist, double estRate, double estAccel) {
        this.time = time;
        this.dist = dist;
        this.rate = rate;
        this.estDist = estDist;
        this.estRate = estRate;
        this.estAccel = estAccel;
    }

    public long time;
    public double dist;
    public double rate;
    public double estDist;
    public double estRate;
    public double estAccel;

    static private Record getRecordFromLine(String line) {
        try (Scanner rowScanner = new Scanner(line)) {
            rowScanner.useDelimiter(COMMA_DELIMITER);
            return new Record(
                Long.valueOf(rowScanner.next()),
                Double.valueOf(rowScanner.next()),
                Double.valueOf(rowScanner.next()),
                Double.valueOf(rowScanner.next()),
                Double.valueOf(rowScanner.next()),
                Double.valueOf(rowScanner.next())
            );
        }
    }

    static public List<Record> readData(String filename) {
        List<Record> records = new ArrayList<>();
        try (Scanner scanner = new Scanner(new File(filename));) {
            // Skip the line of column names.
            scanner.nextLine();
            while (scanner.hasNextLine()) {
                try {
                    records.add(getRecordFromLine(scanner.nextLine()));
                } catch (NumberFormatException e) { /* eat the exception, skip this record */ }
            }
        } catch (FileNotFoundException e) {
            ; // Just eat the exception, return whatever records we may have gathered (probably none).
        }

        return records;
    }

    @Override
    public String toString() {
        return "Record [time=" + time + ", dist=" + dist + ", rate=" + rate
            + ", estDist=" + estDist + ", estRate=" + estRate + ", estAccel=" + estAccel
            + "]";
    }
}
