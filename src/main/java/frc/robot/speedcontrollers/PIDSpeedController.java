package frc.robot.speedcontrollers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

public class PIDSpeedController implements SpeedController {
    private final SpeedController other;
    private final PIDController pid;
    private final Supplier<Double> measurement;

    public PIDSpeedController(SpeedController other, Supplier<Double> measurement, double kP, double kI, double kD) {
        this.other = other;
        this.measurement = measurement;
        pid = new PIDController(kP, kI, kD);
	}

	@Override
    public void pidWrite(double output) {
        other.pidWrite(output);
    }

    @Override
    public void set(double speed) {
        other.setVoltage(pid.calculate(measurement.get(), speed));
    }

    @Override
    public double get() {
        return other.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        other.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return other.getInverted();
    }

    @Override
    public void disable() {
        other.disable();
    }

    @Override
    public void stopMotor() {
        other.stopMotor();
    }
}
