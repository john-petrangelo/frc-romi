package frc.robot.commands.calibration;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class FindMinVoltageToStartMoving extends CommandBase {
    public enum Side {
        LEFT_FWD, LEFT_BACK, RIGHT_FWD, RIGHT_BACK
    };

    private static final long STEP_DURATION_MS = 200;
    private static final double STEP_SIZE_VOLTS = 0.05;

    private RomiDrivetrain drivetrain;
    private long initTime;
    private long stepStartTime;
    private double currentStepVoltage;

    private Supplier<Double> distance;
    private Consumer<Double> drive;

    /**
     * Creates a command that finds the minimum voltage needed to move the robot.
     * <p>
     * Starts with very low voltage and slow ramp up until
     * motion is detected. Once motion is detected, stop and report the voltage.
     */
    public FindMinVoltageToStartMoving(Side side, RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        switch(side) {
            case LEFT_FWD:
            distance = () -> drivetrain.getLeftDistanceInches();
            drive = (volts) -> drivetrain.voltDriveLeft(volts);
            break;
        case LEFT_BACK:
            distance = () -> -drivetrain.getLeftDistanceInches();
            drive = (volts) -> drivetrain.voltDriveLeft(-volts);
            break;
        case RIGHT_FWD:
            distance = () -> drivetrain.getRightDistanceInches();
            drive = (volts) -> drivetrain.voltDriveRight(volts);
            break;
        case RIGHT_BACK:
            distance = () -> -drivetrain.getRightDistanceInches();
            drive = (volts) -> drivetrain.voltDriveRight(-volts);
            break;
        }

        setName(String.format("%s(%s)", "MinVoltsToStartMoving", side.toString()));
    }

    @Override
    public void initialize() {
        super.initialize();
        initTime = System.currentTimeMillis();
        stepStartTime = initTime;
        currentStepVoltage = 0.0;

        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        // If it's too soon, don't bother checking.
        final long now = System.currentTimeMillis();

        // Have we tried this voltage long enough?
        if (now >= stepStartTime + STEP_DURATION_MS) {
            stepStartTime += STEP_DURATION_MS;
            currentStepVoltage += STEP_SIZE_VOLTS;

            System.out.printf("%3d %s Stepping voltage V=%5.3f dist=%5.3f\n",
                now - initTime, getName(), currentStepVoltage,
                distance.get());
        }

        drive.accept(currentStepVoltage);
        return;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%s end voltage=%5.3f distance=%5.3f\n",
            getName(), currentStepVoltage, distance.get());
        SmartDashboard.putNumber("Calibration/" + getName(), currentStepVoltage);

        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return distance.get() > 0.001;
    }
}
