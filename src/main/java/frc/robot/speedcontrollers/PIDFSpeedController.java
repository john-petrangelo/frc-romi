package frc.robot.speedcontrollers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDFSpeedController implements SpeedController {
    private final SpeedController other;
    private final SimpleMotorFeedforward fwdFF;
    private final SimpleMotorFeedforward backFF;
    private final PIDController fwdPID;
    private final PIDController backPID;
    private final Supplier<Double> measurement;

    public PIDFSpeedController(SpeedController other, Supplier<Double> measurement,
            double kSFwd, double kVFwd, double kSBack, double kVBack,
            double kPFwd, double kPBack) {
        this.other = other;
        this.measurement = measurement;
        fwdPID = new PIDController(kPFwd, 0.0, 0.0);
        backPID = new PIDController(kPBack, 0.0, 0.0);
        fwdFF  = new SimpleMotorFeedforward(kSFwd,  kVFwd);
        backFF = new SimpleMotorFeedforward(kSBack, kVBack);
	}

	@Override
    public void pidWrite(double output) {
        other.pidWrite(output);
    }

    @Override
    public void set(double speed) {
        if (speed > 0.0) {
            double ffV = fwdFF.calculate(speed);
            double pidV = fwdPID.calculate(measurement.get(), speed);
            other.setVoltage(ffV + pidV);
            SmartDashboard.putNumber("FF volts", ffV);
            SmartDashboard.putNumber("PID volts", pidV);
        } else {
            other.setVoltage(backFF.calculate(speed) + backPID.calculate(measurement.get(), speed));
        }
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
