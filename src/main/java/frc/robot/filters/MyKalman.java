package frc.robot.filters;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RomiDrivetrain;

public class MyKalman {
    private final KalmanFilter kf;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    /**
     * Kalman filter
     * State vector = [left pos, right pos, left rate, right rate]^T
     */
    public MyKalman(Encoder leftEncoder, Encoder rightEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        reset();

        // state transition function
        double dt = 0.020;
        RealMatrix A = new Array2DRowRealMatrix(new double[][] { 
            { 1, 0, dt,  0 },
            { 0, 1,  0, dt },
            { 0, 0,  1,  0 },
            { 0, 0,  0,  1 },
        });

        // control input
        RealMatrix B = null;

        // measurement function
        RealMatrix H = new Array2DRowRealMatrix(new double[][] {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
        });

        // process covariance (none)
        RealMatrix Q = new Array2DRowRealMatrix(new double[][] { 
            { 0, 0, 0, 0},
            { 0, 0, 0, 0},
            { 0, 0, 0, 0},
            { 0, 0, 0, 0},
        });

        // measure noise covariance (none)
        RealMatrix R = new Array2DRowRealMatrix(new double[][] {
            { 0, 0 },
            { 0, 0 },
        });

        // initial state means and covariances (assume identity matrix for now)
        RealVector X0 = new ArrayRealVector(new double[] { 0, 0, 0, 0 });
        RealMatrix P0 = new Array2DRowRealMatrix(new double[][] {
            { 1, 0, 0, 0},
            { 0, 1, 0, 0},
            { 0, 0, 1, 0},
            { 0, 0, 0, 1},
        });

        ProcessModel processModel = new DefaultProcessModel(A, B, Q, X0, P0);
        MeasurementModel measurementModel = new DefaultMeasurementModel(H, R);

        kf = new KalmanFilter(processModel, measurementModel);
    }

    /**
     * Predict step
     */
    public void predict() {
        // Alt. kf.predict(u);
        kf.predict();
    }

    /**
     * Gets the latest value from the encoder and updates internal state of the filter.
     */
    public void update() {
        RealVector z = new ArrayRealVector(new double[] {
            leftEncoder.getDistance(), rightEncoder.getDistance()
        } );

        kf.correct(z);

        // SmartDashboard.putNumber("MyKalman/distance", estDistance);
        // SmartDashboard.putNumber("MyKalman/rate", estRate);
    }

    /**
     * Returns the estimated left position.
     */
    public double getLeftPosition() {
        return kf.getStateEstimation()[0];
    }

    /**
     * Returns the estimated right position.
     */
    public double getRightPosition() {
        return kf.getStateEstimation()[1];
    }

    /**
     * Returns the estimated left position.
     */
    public double getLeftRate() {
        return kf.getStateEstimation()[2];
    }

    /**
     * Returns the estimated right rate.
     */
    public double getRightRate() {
        return kf.getStateEstimation()[3];
    }

    /**
     * Returns the left Encoder.
     */
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Returns the right Encoder.
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    public void reset() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
}
