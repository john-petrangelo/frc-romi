package frc.robot.speedcontrollers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class PIDFSpeedController implements SpeedController {
    private final SpeedController other;
    private final SimpleMotorFeedforward fwdFF;
    private final SimpleMotorFeedforward backFF;
    private final PIDController pid;
    private final Supplier<Double> measurement;

    public PIDFSpeedController(SpeedController other, Supplier<Double> measurement,
            double kSFwd, double kVFwd, double kSBack, double kVBack,
            double kP, double kI, double kD) {
        this.other = other;
        this.measurement = measurement;
        pid = new PIDController(kP, kI, kD);
        fwdFF  = new SimpleMotorFeedforward(kSFwd,  kVFwd);
        backFF = new SimpleMotorFeedforward(kSBack, kVBack);
	}

	@Override
    public void pidWrite(double output) {
        other.pidWrite(output);
    }

    @Override
    public void set(double speed) {
        double ffVoltage;

        if (speed > 0.0) {
            ffVoltage = fwdFF.calculate(100*speed);
        } else {
            ffVoltage = backFF.calculate(100*speed);
        }

        final double pidVoltage = pid.calculate(measurement.get(), speed);

        other.setVoltage(ffVoltage + pidVoltage);
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
