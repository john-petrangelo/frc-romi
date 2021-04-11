package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveVoltsGetRate extends CommandBase {
    private RomiDrivetrain drivetrain;
    private double volts;
    private LinearFilter leftRateFilter = LinearFilter.movingAverage(20);
    private LinearFilter rightRateFilter = LinearFilter.movingAverage(20);

    /**
     * Creates a command that drives at the specified voltage saving current rate into Smart Dashboard.
     * <p>
     * Algorithm details TBD.
     */
    public DriveVoltsGetRate(double volts, RomiDrivetrain drivetrain) {
        this.volts = volts;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.printf("%s Driving at %5.3f volts\n", getName(), volts);

        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        double filteredLeftRate = leftRateFilter.calculate(drivetrain.getLeftRate());
        double filteredRightRate = rightRateFilter.calculate(drivetrain.getRightRate());

        SmartDashboard.putNumber("Calibration/LeftRateForVolts(" + volts + ")", drivetrain.getLeftRate());
        SmartDashboard.putNumber("Calibration/RightRateForVolts(" + volts + ")", drivetrain.getRightRate());
        SmartDashboard.putNumber("Calibration/FilteredLeftRateForVolts(" + volts + ")", filteredLeftRate);
        SmartDashboard.putNumber("Calibration/FilteredRightRateForVolts(" + volts + ")", filteredRightRate);

        drivetrain.voltDriveLeft(volts);
        drivetrain.voltDriveRight(volts);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%s Done driving at %5.3f volts\n", getName(), volts);

        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
