package frc.robot.filters;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RomiDrivetrain;

public class TrashCompactor {
    private final double MAX_SPEED = RomiDrivetrain.MAX_SPEED * 1.1;

    private final Encoder encoder;

    private long lastSampleTimeMS;
    private double lastSampleDistance;

    private double estDistance;
    private double estRate;

    int sameCount;

    /**
     * Filter designed to throw out the Romi encoders' repeating data and only use
     * measurements that have changed since last time.
     *
     * Algorithm: If the encoder distance changes and the rate is "reasonable"
     * then trust the measurement, otherwise trust the prediction. If 10 samples
     * in a row have the same value then take it as a new value (in case we the
     * robot really is stopped).
     */
    public TrashCompactor(Encoder encoder) {
        this.encoder = encoder;
        reset();

        this.lastSampleTimeMS = System.currentTimeMillis();
        this.lastSampleDistance = encoder.getDistance();

        this.estDistance = 0.0;
        this.estRate = 0.0;

        sameCount = 0;
    }

    /**
     * Alternate constructor for unit tests. Instead of encoder, take initial distance and rate.
     */
    public TrashCompactor(long initialTime, double initialDistance, double initialRate,
            double estDistance, double estRate) {
        this.encoder = null;

        this.lastSampleTimeMS = initialTime;
        this.lastSampleDistance = initialDistance;

        this.estDistance = estDistance;
        this.estRate = estRate;
    }

    /**
     * Gets the latest value from the encoder and updates internal state of the filter.
     */
    public void update() {
        final long now = System.currentTimeMillis();
        final double encoderDistance = encoder.getDistance();
        final double encoderRate = encoder.getRate();

        update(now, encoderDistance, encoderRate);

        SmartDashboard.putNumber("EncoderEstimater/distance", estDistance);
        SmartDashboard.putNumber("EncoderEstimater/rate", estRate);
    }

    /**
     * Update the internal state of the filter.
     */
    public void update(long sampleTimeMS, double encoderDistance, double encoderRate) {
        final double dt = (sampleTimeMS - lastSampleTimeMS) / 1000.0;

        // Special case, duplicate time value or initial sample.
        if (dt == 0.0) {
            lastSampleTimeMS = sampleTimeMS;
            return;
        }

        // Prediction step.
        final double predDistance = estDistance + estRate * dt;
        final double predRate     = estRate; // Assume rate is constant.

        // Determine the quality of the sample.
        final boolean rateIsGood = Math.abs(encoderRate) < MAX_SPEED;
        final boolean isMoving = encoderRate != 0.0;
        final boolean haveNewData = sameCount >= 10 || encoderDistance != lastSampleDistance;

        // Update step if a new value has arraive and the rate is reasonable.
        if ((rateIsGood && haveNewData) || !isMoving) {
            // Trust the measurement 100% (G and H both 1.0).
            estDistance = encoderDistance;
            estRate = encoderRate;
            lastSampleDistance = encoderDistance;
            sameCount = 0;
        } else {
            // Trust the measurement 0% (G and H both 0.0).
            estDistance = predDistance;
            estRate = predRate;
            sameCount++;
        }

        lastSampleTimeMS = sampleTimeMS;
    }

    /**
     * Returns the current distance estimate.
     */
    public double getDistance() {
        return estDistance;
    }

    /**
     * Returns the current rate estimate.
     */
    public double getRate() {
        return estRate;
    }

    public void reset() {
        encoder.reset();
        estDistance = 0;
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
