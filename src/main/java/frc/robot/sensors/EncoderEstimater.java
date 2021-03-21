package frc.robot.sensors;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EncoderEstimater {
    private final Encoder encoder;

    private long lastKnownDistanceTime;
    private double lastKnownDistance;
    private double lastKnownRate;

    private double modelDistance;
    private double modelRate;
    private double modelAccel;

    public EncoderEstimater(Encoder other) {
        encoder = other;
        reset();

        lastKnownDistanceTime = System.currentTimeMillis();

        lastKnownDistance = encoder.getDistance();
        lastKnownRate = encoder.getRate();

        modelDistance = 0.0;
        modelRate = 0.0;
        modelAccel = 0.0;
    }

    /**
     * Gets the latest value from the encoder and updates internal state.
     */
    public void update() {
        final long now = System.currentTimeMillis();
        final double dt = (now - lastKnownDistanceTime) / 1000.0;
        final double encoderDistance = encoder.getDistance();
        
        if (encoderDistance != lastKnownDistance) {
            // New encoder distance available. Update the model using the measured value.
            modelDistance = encoderDistance;
            modelRate = (modelDistance - lastKnownDistance) / dt;
            modelAccel = (modelRate - lastKnownRate) / dt;

            lastKnownRate = modelRate;
            lastKnownDistance = modelDistance;
            lastKnownDistanceTime = now;
        } else {
            // Encoder value didn't change, estimate new model values.
            modelDistance = lastKnownDistance + modelRate * dt;
            modelRate = lastKnownRate + modelAccel * dt;
        }

        System.out.println(String.format("DATA,%d,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f",
            now, encoder.getDistance(), encoder.getRate(), modelDistance, modelRate, modelAccel));

        SmartDashboard.putNumber("EncoderEstimater/distance", modelDistance);
        SmartDashboard.putNumber("EncoderEstimater/rate", modelRate);
        SmartDashboard.putNumber("EncoderEstimater/accel", modelAccel);
    }

    public double getDistance() {
        return modelDistance;
    }

    public double getRate() {
        return modelRate;
    }

    public double getAccel() {
        return modelAccel;
    }

    public void reset() {
        encoder.reset();
    }

    /**
     * Returns the encoder distance as reported directly by the wrapped Encoder.
     */
    public double getEncoderDistance() {
        return encoder.getDistance();
    }

    /**
     * Returns the encoder rate as reported directly by the wrapped Encoder.
     */
    public double getEncoderRate() {
        return encoder.getRate();
    }
}
