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
        double ffV;
        double pidV;
        if (speed > 0.0) {
            ffV = fwdFF.calculate(speed);
            pidV = fwdPID.calculate(measurement.get(), speed);
            System.out.printf("PIDF[F%s] speed=%4.2f ffV=%5.3f pidV=%5.3f actual=%4.2f\n",
                name, speed, ffV, pidV, measurement.get());
        } else if (speed < 0.0) {
            ffV = backFF.calculate(speed);
            pidV = backPID.calculate(measurement.get(), speed);
            System.out.printf("PIDF[F%s] speed=%4.2f ffV=%5.3f pidV=%5.3f actual=%4.2f\n",
                name, speed, ffV, pidV, measurement.get());
        } else {
            ffV = backFF.calculate(speed);
            pidV = backPID.calculate(measurement.get(), speed);
        }

        other.setVoltage(ffV + pidV);
        SmartDashboard.putNumber(name + "-FF volts", ffV);
        SmartDashboard.putNumber(name + "-PID volts", pidV);
        SmartDashboard.putNumber(name + "-requested-speed", speed);
        SmartDashboard.putNumber(name + "-actual-speed", Math.signum(speed) * measurement.get());
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
