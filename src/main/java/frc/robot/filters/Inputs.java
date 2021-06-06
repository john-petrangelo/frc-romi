package frc.robot.filters;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * Control inputs are linear rate and turn rate (fwd/back joystick, left/right joystick).
 * u = [linearRate, turnRate]^T
 */ 
class Inputs {
    Matrix<N2,N1> m;
    final int ROW_LINEAR_RATE = 0;
    final int ROW_TURN_RATE = 1;

    Inputs(double linearRate, double turnRate) {
        m = new Matrix<>(Nat.N2(), Nat.N1());
        setLinearRate(linearRate);
        setTurnRate(turnRate);
    }

    Inputs(Matrix<N2, N1> m) {
        this.m = m.copy();
    }

    double getLinearRate() {
        return m.get(ROW_LINEAR_RATE, 0);
    }

    double getTurnRate() {
        return m.get(ROW_TURN_RATE, 0);
    }

    void setLinearRate(double r) {
        m.set(ROW_LINEAR_RATE, 0, r);
    }

    void setTurnRate(double r) {
        m.set(ROW_TURN_RATE, 0, r);
    }

    /**
     * Get the radius of curvature based on the linear and turn rates.
     */
    double getRadius() {
        if (getTurnRate() < 0.001) {
            return 1e9;
        } else {
            return getLinearRate() / Math.toRadians(getTurnRate());
        }
    }

    @Override
    public String toString() {
        return String.format("Inputs [linearRate=%4.2f, turnRate=%4.2f]",
            getLinearRate(), getTurnRate());
    }
}
