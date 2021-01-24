package frc.robot.speedcontrollers;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class FeedforwardSpeedController implements SpeedController {
    private final SpeedController other;
    private final SimpleMotorFeedforward fwdFF;
    private final SimpleMotorFeedforward backFF;

    public FeedforwardSpeedController(SpeedController other, double kSFwd, double kVFwd, double kSBack, double kVBack) {
        this.other = other;
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
            other.setVoltage(fwdFF.calculate(speed));
        } else {
            other.setVoltage(backFF.calculate(speed));
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
