package frc.robot.filters;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

/**
 * States tracks robot state for X position, Y position, and heading (theta).
 * x = [xPos, yPos, theta]^T
 */
class States {
    final int ROW_X = 0;
    final int ROW_Y = 1;
    final int ROW_THETA = 2;

    Matrix<N3,N1> m;

    States(double x, double y, double theta) {
        m = new Matrix<>(Nat.N3(), Nat.N1());
        setX(x);
        setY(y);
        setTheta(theta);
    }

    States(Matrix<N3, N1> m) {
        this.m = m.copy();
    }

    double getX() {
        return m.get(ROW_X, 0);
    }

    double getY() {
        return m.get(ROW_Y, 0);
    }

    double getTheta() {
        return m.get(ROW_THETA, 0);
    }

    void setX(double x) {
        m.set(ROW_X, 0, x);
    }

    void setY(double y) {
        m.set(ROW_Y, 0, y);
    }

    void setTheta(double theta) {
        m.set(ROW_THETA, 0, theta);
    }

    Matrix<N3, N1> getMatrix() {
        return m;
    }

    @Override
    public String toString() {
        return String.format("States [X=%5.3f, Y=%5.3f, theta=%3.1f]", getX(), getY(), getTheta());
    }
}
