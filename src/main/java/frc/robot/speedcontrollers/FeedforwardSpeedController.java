package frc.robot.speedcontrollers;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeedforwardSpeedController implements SpeedController {
    private final SpeedController other;
    private SimpleMotorFeedforward fwdFF;
    private SimpleMotorFeedforward backFF;
    private final String name;

    public FeedforwardSpeedController(String name, SpeedController other,
            double kSFwd, double kVFwd, double kSBack, double kVBack) {
        this.name = name;
        this.other = other;
        setParameters(kSFwd, kVFwd, kSBack, kVBack);
    }

    public void setParameters(double kSFwd, double kVFwd, double kSBack, double kVBack) {
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
            other.setVoltage(ffV);
            System.out.printf("FF[F%s] speed=%5.3f ffV=%5.3f\n", name, speed, ffV);
            SmartDashboard.putNumber(name + "-FF volts", ffV);
            SmartDashboard.putNumber(name + "-PID volts", 0);
        } else {
            double ffV = backFF.calculate(speed);
            other.setVoltage(ffV);
            System.out.printf("FF[B%s] speed=%5.3f ffV=%5.3f\n", name, speed, ffV);
            SmartDashboard.putNumber(name + "-FF volts", ffV);
            SmartDashboard.putNumber(name + "-PID volts", 0);
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
