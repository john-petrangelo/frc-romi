package frc.robot.filters;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * Outputs represents the measurements (sensors) we track on the robot.
 * We are currently tracking the rate of the left and right wheels in
 * inches per second.
 *
 * z = [leftRate, rightRate]^T
 */
class Outputs {
    Matrix<N2,N1> m;
    final int ROW_LEFT_RATE = 0;
    final int ROW_RIGHT_RATE = 1;

    Outputs(double leftRate, double rightRate) {
        m = new Matrix<>(Nat.N2(), Nat.N1());
        setLeftRate(leftRate);
        setRightRate(rightRate);
    }

    Outputs(Matrix<N2, N1> m) {
        this.m = m.copy();
    }

    double getLeftRate() {
        return m.get(ROW_LEFT_RATE, 0);
    }

    double getRightRate() {
        return m.get(ROW_RIGHT_RATE, 0);
    }

    void setLeftRate(double d) {
        m.set(ROW_LEFT_RATE, 0, d);
    }

    void setRightRate(double d) {
        m.set(ROW_RIGHT_RATE, 0, d);
    }

    Matrix<N2, N1> getMatrix() {
        return m;
    }

    @Override
    public String toString() {
        return "Outputs [leftRate=" + getLeftRate() + ", rightRate=" + getRightRate() + "]";
    }
}
