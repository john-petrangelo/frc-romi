package frc.robot.speedcontrollers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDFSpeedController implements SpeedController {
    private final SpeedController other;
    private SimpleMotorFeedforward fwdFF;
    private SimpleMotorFeedforward backFF;
    private PIDController fwdPID;
    private PIDController backPID;
    private final Supplier<Double> measurement;
    private final String name;

    public PIDFSpeedController(String name, SpeedController other, Supplier<Double> measurement,
            double kSFwd, double kVFwd, double kPFwd,
            double kSBack, double kVBack, double kPBack) {
        this.name = name;
        this.other = other;
        this.measurement = measurement;
        setParameters(kSFwd, kVFwd, kPFwd, kSBack, kVBack, kPBack);
	}

    public void setParameters(double kSFwd, double kVFwd, double kPFwd,
                              double kSBack, double kVBack, double kPBack) {
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
            System.out.printf("PIDF[F%s] speed=%5.3f ffV=%5.3f pidV=%5.3f\n", name, speed, ffV, pidV);
            SmartDashboard.putNumber(name + "-FF volts", ffV);
            SmartDashboard.putNumber(name + "-PID volts", pidV);
        } else if (speed < 0.0) {
            double ffV = backFF.calculate(speed);
            double pidV = backPID.calculate(measurement.get(), speed);
            other.setVoltage(ffV + pidV);
            System.out.printf("PIDF[B%s] speed=%5.3f ffV=%5.3f pidV=%5.3f\n", name, speed, ffV, pidV);
            SmartDashboard.putNumber(name + "-FF volts", ffV);
            SmartDashboard.putNumber(name + "-PID volts", pidV);
        } else {
            double ffV = backFF.calculate(speed);
            double pidV = backPID.calculate(measurement.get(), speed);
            other.setVoltage(ffV + pidV);
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
