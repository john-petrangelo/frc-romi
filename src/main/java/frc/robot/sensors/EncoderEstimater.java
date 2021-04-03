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
     * Alternate constructor for unit tests. Instead of encoder, take initial distance and rate.
     */
    public EncoderEstimater(long initialTime, double initialDistance, double initialRate,
            double modelDistance, double modelRate, double modelAccel) {
        this.encoder = null;

        lastKnownDistanceTime = initialTime;
        lastKnownDistance = initialDistance;
        lastKnownRate = initialRate;

        this.modelDistance = modelDistance;
        this.modelRate = modelRate;
        this.modelAccel = modelAccel;
    }

    /**
     * Gets the latest value from the encoder and updates internal state of the filter.
     */
    public void update() {
        final long now = System.currentTimeMillis();
        final double encoderDistance = encoder.getDistance();
        final double encoderRate = encoder.getRate();

        update(now, encoderDistance, encoderRate);

        System.out.println(String.format("DATA,%d,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f",
            now, encoderDistance, encoderRate, modelDistance, modelRate, modelAccel));

        SmartDashboard.putNumber("EncoderEstimater/distance", modelDistance);
        SmartDashboard.putNumber("EncoderEstimater/rate", modelRate);
        SmartDashboard.putNumber("EncoderEstimater/accel", modelAccel);
    }

    /**
     * Update the internal state of the filter.
     */
    public void update(long nowMS, double encoderDistance, double encoderRate) {
        final double dt = (nowMS - lastKnownDistanceTime) / 1000.0;
        
        if (encoderDistance != lastKnownDistance) {
            // New encoder distance available. Update the model using the measured value.
            modelDistance = encoderDistance;
            modelRate = (modelDistance - lastKnownDistance) / dt;
            modelAccel = (modelRate - lastKnownRate) / dt;
            modelAccel = 0;

            lastKnownRate = modelRate;
            lastKnownDistance = modelDistance;
            lastKnownDistanceTime = nowMS;
        } else {
            // Encoder value didn't change, predict new model values.
            modelDistance = lastKnownDistance + modelRate * dt;
            modelRate = lastKnownRate + modelAccel * dt;
        }
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
