package frc.robot.filters;

import edu.wpi.first.wpilibj.estimator.UnscentedKalmanFilter;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;

/**
 * RomiPoseEstimator tracks the location and heading of a Romi robot.
 * Control inputs and sensor outputs are fed into the filter to track
 * the state of the robot.
 */
public class RomiPoseEstimator {
    // 3 states, 2 inputs, 2 outputs
    private final UnscentedKalmanFilter<N3, N2, N2> ukf;

    /**
     * Kalman filter<p>
     *
     * <pre>
     * States: x = [xPos, yPos, heading]]^T
     * Outputs: z = [leftWheelRate, rightWheelRate]^T
     * Inputs: u = [linearRate, turnRate]^T
     * </pre>
     * Distances are measured in inches. Angles are measured in degrees.
     */
    public RomiPoseEstimator() {
        // The error standard deviations used to create the process covariance matrix.
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(
            Math.sqrt(0.10),
            Math.sqrt(0.10),
            Math.sqrt(0.09));

        // The error standard deviations used to create the measurement covariance matrix.
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(
            Math.sqrt(0.14),
            Math.sqrt(0.14));

        ukf = new UnscentedKalmanFilter<>(
            Nat.N3(),           /* number of states */
            Nat.N2(),           /* number of outputs (sensors) */
            this::f,            /* state transition function */
            this::h,            /* measurement function */
            stateStdDevs,       /* process covariance */
            measurementStdDevs, /* measurement noise covariance */
            0.020               /* nominalDtSeconds */);
    }

    /**
     * State transition function. Performs the derivative of the state vector.
     * 
     * @returns change in state.
     */
    private Matrix<N3, N1> f(Matrix<N3,N1> states, Matrix<N2,N1> inputs) {
        States x = new States(states);
        Inputs u = new Inputs(inputs);

        // Calculate the change in state.
        double thetaRad = Math.toRadians(x.getTheta());
        States dState = new States(
            u.getLinearRate() * Math.sin(thetaRad),
            u.getLinearRate() * Math.cos(thetaRad),
            u.getTurnRate());

        return dState.getMatrix();
    };

    /**
     * Measurement function, maps the current state and inputs into measurement space.
     * The result of this function can be compared directly  to the actual measurements
     * provided by the sensors.
     * 
     * @return state and inputs mapped to measurement space
     */
    private Matrix<N2,N1> h(Matrix<N3,N1> states, Matrix<N2,N1> inputs) {
        System.out.println("Called h");

        // Width, distance between the wheels of the robot. Should get this from a RobotMap class.
        final double width = 2.75;

        // Since we are not tracking rates in the state, the state has no contribution.
        // States x = new States(states);

        // Inputs 
        Inputs u = new Inputs(inputs);

        // The offset is used to find the inner and outer wheel turn radii.
        double offset = width / (2 * u.getRadius());

        Outputs outputs = new Outputs(
            (1 - offset) * u.getLinearRate(),
            (1 + offset) * u.getLinearRate()
        );
        return outputs.getMatrix();
    };

    /**
     * Predict step
     */
    public void predict(double linearRate, double turnRate, double dt) {
        Matrix<N2, N1> u = VecBuilder.fill(linearRate, turnRate);
        ukf.predict(u, dt);
    }

    /**
     * Updates (corrects) the internal state of the filter based on the
     * current control inputs and the measured outputs from the sensors.
     */
    public void update(
        // Inputs
        double linearRate, double turnRate,
        // Outputs (measurements)
        double leftEncoderRate, double rightEncoderRate)
    {
        Matrix<N2, N1> u = VecBuilder.fill(linearRate, turnRate);
        Matrix<N2, N1> z = VecBuilder.fill(leftEncoderRate, rightEncoderRate);

        ukf.correct(u, z);
    }

    /**
     * Get the estimated position X value.
     */
    public double getEstX() {
        return ukf.getXhat(0);
    }

    /**
     * Get the estimated position Y value.
     */
    public double getEstY() {
        return ukf.getXhat(1);
    }

    /**
     * Get the estimated heading.
     */
    public double getEstHeading() {
        return ukf.getXhat(2);
    }

    /**
     * Get position X variance
     */
    public double getVarX() {
        return ukf.getP(0, 0);
    }

    /**
     * Get position Y variance
     */
    public double getVarY() {
        return ukf.getP(1, 1);
    }

    /**
     * Get position heading variance
     */
    public double getVarHeading() {
        return ukf.getP(2, 2);
    }

    /**
     * A string representation of the complete current state and covariance matrix of the Pose.
     */
    @Override
    public String toString() {
        Matrix<N3, N3> P = ukf.getP();
        String result = String.format(
            "Pose [Est[%4.2f, %4.2f, %4.2f]  P[[%4.2f, %4.2f, %4.2f], [%4.2f, %4.2f, %4.2f], [%4.2f, %4.2f, %4.2f]]",
            getEstX(), getEstY(), getEstHeading(),
            P.get(0, 0), P.get(0, 1), P.get(0, 2),
            P.get(1, 0), P.get(1, 1), P.get(1, 2),
            P.get(2, 0), P.get(2, 1), P.get(2, 2)
        );

        return result;
    }

    /**
     * The current state and variances in CSV format:
     *   Est X, Est Y, Est Heading, Var X, Var Y, Var Heading
     */
    public String toCSV() {
        String result = String.format(
            "%f,%f,%f,%f,%f,%f",
            getEstX(), getEstY(), getEstHeading(),
            getVarX(), getVarY(), getVarHeading());
        return result;
    }
}
