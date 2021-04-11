package frc.robot.commands.calibration;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class CalibrateFFConstants extends CommandBase {
    private enum Phase {
        DRIVE_SLOW, DRIVE_FAST, DONE
    }

    private static final double SLOW_VOLTAGE = 2.0;
    private static final double FAST_VOLTAGE = 4.0;

    private RomiDrivetrain drivetrain;
    private Supplier<Double> distance;
    private Phase phase = Phase.DRIVE_SLOW;

    /**
     * Creates a command that finds the feed forward constants kS and kV.
     * <p>
     * Algorithm details TBD.
     */
    public CalibrateFFConstants(RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // distance = () -> drivetrain.getLeftDistanceInches();
        // distance = () -> drivetrain.getRightDistanceInches();
    }

    @Override
    public void initialize() {
        super.initialize();

        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {

        double volts;
        switch (phase) {
            case DRIVE_SLOW:
                volts = SLOW_VOLTAGE;
                break;
            case DRIVE_FAST:
                volts = FAST_VOLTAGE;
                break;
            default:
                volts = 0.0;
        }

        drivetrain.voltDriveLeft(volts);
        drivetrain.voltDriveRight(volts);

        return;
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.printf("%s end voltage=%5.3f distance=%5.3f\n",
        //     getName(), currentStepVoltage, distance.get());
        // SmartDashboard.putNumber("Calibration/" + getName(), currentStepVoltage);

        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        // switch (phase) {
        //     case DRIVE_SLOW:
        //         break;
        //     case DRIVE_FAST:
        //         break;
        //     default:
        //         break;
        // }

        // return currentStepVoltage < 0.0;
        return true;
    }
}
