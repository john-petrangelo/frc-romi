package frc.robot.filters;

import org.junit.jupiter.api.Test;

public class RomiPoseEstimatorTest {
    @Test
    public void testStuff() {
        RomiPoseEstimator pose = new RomiPoseEstimator();
        double dt = 0.020;

        for (int i = 0; i < 50; i++) {
            pose.predict(10.0, 0.0, dt);
            System.out.println(pose.toCSV());
        }

        for (int i = 0; i < 50; i++) {
            pose.predict(10.0, 90.0, dt);
            System.out.println(pose.toCSV());
        }

        for (int i = 0; i < 50; i++) {
            pose.predict(2.0, -90.0, dt);
            System.out.println(pose.toCSV());
        }
    }
}
