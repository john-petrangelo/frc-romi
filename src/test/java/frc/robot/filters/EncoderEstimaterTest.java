package frc.robot.filters;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

public class EncoderEstimaterTest {
    static private String FILENAME = "data-accel-model.csv";
    static private double TOLERANCE = 0.001;

    // @Test
    // public void testUpdate() {
    //     List<Record> records = Record.readData(FILENAME);

    //     // Create an EncoderEstimater, using the first data record for initial conditions.
    //     Record firstRecord = records.remove(0);
    //     EncoderEstimater ee = new EncoderEstimater(0, firstRecord.dist, firstRecord.rate,
    //         firstRecord.estDist, firstRecord.estRate, firstRecord.estAccel);

    //     records.forEach((r) -> {
    //         String msg = "at " + r;
    //         ee.update(r.time, r.dist, r.rate);

    //         assertEquals(r.estDist, ee.getDistance(), TOLERANCE, msg + " dist");
    //         assertEquals(r.estRate, ee.getRate(), TOLERANCE, msg + " rate");
    //         assertEquals(r.estAccel, ee.getAccel(), TOLERANCE, msg + " accel");
    //     });
    // }

    @Test
    public void testAllNewDistances() {
        // Initial conditions - at origin, stopped, no acceleration.
        EncoderEstimater ee = new EncoderEstimater(0, 0.0, 0.0, 0.0, 0.0, 0.0);
        long t = 0;

        // Step 20ms, new position = 1.0
        ee.update(t += 20, 1.0, 99);
        assertEquals(1.0, ee.getDistance(), "dist");
        assertEquals(50.0, ee.getRate(), "rate");
        assertEquals(2500.0, ee.getAccel(), "accel");

        // Step 20ms, new position = 2.0
        ee.update(t += 20, 2.0, 99);
        assertEquals(2.0, ee.getDistance(), "dist");
        assertEquals(50.0, ee.getRate(), "rate");
        assertEquals(0.0, ee.getAccel(), "accel");

        // Step 20ms, new position = 4.0
        ee.update(t += 20, 4.0, 99);
        assertEquals(4.0, ee.getDistance(), "dist");
        assertEquals(100.0, ee.getRate(), "rate");
        assertEquals(2500.0, ee.getAccel(), "accel");

        // Step 20ms, new position = 5.0
        ee.update(t += 20, 5.0, 99);
        assertEquals(5.0, ee.getDistance(), "dist");
        assertEquals(50.0, ee.getRate(), "rate");
        assertEquals(-2500.0, ee.getAccel(), "accel");

        // Step 20ms, new position = 6.0
        ee.update(t += 20, 6.0, 99);
        assertEquals(6.0, ee.getDistance(), "dist");
        assertEquals(50.0, ee.getRate(), "rate");
        assertEquals(0.0, ee.getAccel(), "accel");
    }

    @Test
    public void testRepeatedDistances() {
        // Initial conditions - at origin, stopped, no acceleration.
        EncoderEstimater ee = new EncoderEstimater(0, 0.0, 0.0, 0.0, 0.0, 0.0);
        long t = 0;

        // Step 20ms, repeat position = 0.0
        ee.update(t += 20, 0.0, 99);
        assertEquals(0.0, ee.getDistance(), "dist");
        assertEquals(0.0, ee.getRate(), "rate");
        assertEquals(0.0, ee.getAccel(), "accel");

        // Step 20ms, new position = 1.0
        ee.update(t += 20, 1.0, 99);
        assertEquals(1.0, ee.getDistance(), "dist");
        assertEquals(25.0, ee.getRate(), "rate");
        assertEquals(625.0, ee.getAccel(), "accel");

        // Step 20ms, repeat position = 1.0
        ee.update(t += 20, 1.0, 99);
        assertEquals(1.5, ee.getDistance(), "dist");
        assertEquals(37.5, ee.getRate(), "rate");
        assertEquals(625.0, ee.getAccel(), "accel");

        // Step 20ms, repeat position = 1.0
        ee.update(t += 20, 1.0, 99);
        assertEquals(2.5, ee.getDistance(), "dist");
        assertEquals(50.0, ee.getRate(), "rate");
        assertEquals(625.0, ee.getAccel(), "accel");
    }
}
