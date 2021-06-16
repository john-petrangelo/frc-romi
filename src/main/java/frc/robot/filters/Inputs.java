package frc.robot.filters;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Inputs are the requested controls input into the robot, e.g. joystick controls.
 */ 
class Inputs {
    Matrix<N2,N1> m;
    final int ROW_LINEAR_RATE = 0;
    final int ROW_TURN_RATE = 1;

    /**
     * Create an Inputs object.
     * 
     * Control inputs are linear rate and turn rate (typically fwd/back joystick, left/right joystick).
     * u = [linearRate, turnRate]^T
     */
    Inputs(double linearRate, double turnRate) {
        m = new Matrix<>(Nat.N2(), Nat.N1());
        setLinearRate(linearRate);
        setTurnRate(turnRate);
    }

    /**
     * Create an Inputs object from the equivalent single column matrix.
     */
    Inputs(Matrix<N2, N1> m) {
        this.m = m.copy();
    }

    /**
     * Get the linear/arc rate of the robot in inches per second.
     */
    double getLinearRate() {
        return m.get(ROW_LINEAR_RATE, 0);
    }

    /**
     * Get the turn rate of the robot in degrees per second.
     */
    double getTurnRate() {
        return m.get(ROW_TURN_RATE, 0);
    }

    /**
     * Set the linear/arc rate of the robot in inches per second.
     */
    void setLinearRate(double r) {
        m.set(ROW_LINEAR_RATE, 0, r);
    }

    /**
     * Set the turn rate of the robot in degrees per second.
     */
    void setTurnRate(double r) {
        m.set(ROW_TURN_RATE, 0, r);
    }

    /**
     * Get the radius of curvature based on the linear and turn rates.
     */
    double getRadius() {
        if (getTurnRate() < 0.001) {
            // If the turn rate is very slow then assume an effectively infinite radius.
            return 1e9;
        } else {
            // Radius = d / beta, where d is the arc speed and beta is the angular rate.
            return getLinearRate() / Math.toRadians(getTurnRate());
        }
    }

    @Override
    public String toString() {
        return String.format("Inputs [linearRate=%4.2f, turnRate=%4.2f]",
            getLinearRate(), getTurnRate());
    }
}
