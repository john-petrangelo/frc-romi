package frc.robot.speedcontrollers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NTVoltsSpeedController implements SpeedController {
    private final SpeedController other;
    private final Supplier<Double> measurement;
    private final String name;
    double volts;

    public NTVoltsSpeedController(String name, SpeedController other, Supplier<Double> measurement,
            double volts) {
        this.name = name;
        this.other = other;
        this.measurement = measurement;
        setParameters(volts);
        SmartDashboard.putNumber("romi-o/set-volts", volts);
	}

    public void setParameters(double volts) {
        System.out.printf("NTVolts[F%s] volts=%4.2f\n", name, volts);
        this.volts = volts;
    }

	@Override
    public void pidWrite(double output) {
        other.pidWrite(output);
    }

    @Override
    public void set(double speed) {
        other.setVoltage(Math.signum(speed) * volts);
        SmartDashboard.putNumber(name + "-requested-speed", speed);
        SmartDashboard.putNumber(name + "-actual-speed", Math.signum(speed) * measurement.get());
        SmartDashboard.putNumber(name + "-volts", Math.signum(speed) * volts);
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
