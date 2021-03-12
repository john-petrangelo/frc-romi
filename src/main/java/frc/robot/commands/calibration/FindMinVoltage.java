package frc.robot.commands.calibration;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class FindMinVoltage extends CommandBase {
    public enum Side {
        LEFT, RIGHT
    };

    private static final long STEP_DURATION_MS = 200;
    private static final double STEP_SIZE_VOLTS = 0.05;

    private RomiDrivetrain drivetrain;
    private long stepStartTime;
    private double currentStepVoltage;
    private double minVoltage = -1.0;

    private Supplier<Double> distance;
    private Consumer<Double> drive;

    /**
     * Creates a command finds the minimum voltage needed to move the robot.
     */
    public FindMinVoltage(Side side, RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        switch(side) {
        case LEFT:
            distance = drivetrain::getLeftDistanceInches;
            drive = drivetrain::voltDriveLeft;
            break;
        case RIGHT:
            distance = drivetrain::getRightDistanceInches;
            drive = drivetrain::voltDriveRight;
            drive = drive.andThen(drivetrain::voltDriveLeft);
            break;
        }

        Consumer<Double> c1 = Math::abs;
        Consumer<Double> c2 = Math::abs;
        Consumer<Double> c3 = c1.andThen(c2);

        c3.accept(5.0);

        setName(String.format("%s(%s)", getName(), side.toString()));
    }

    @Override
    public void initialize() {
        super.initialize();
        stepStartTime = System.currentTimeMillis();
        currentStepVoltage = STEP_SIZE_VOLTS;
        minVoltage = -1.0;

        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        // If it's too soon, don't bother checking.
        long now = System.currentTimeMillis();

        // Did we move?
        if (minVoltage < 0.0 && distance.get() > 0.0) {
            System.out.printf("%3d %s Moved voltage V=%5.3f dist=%5.3f\n",
                now % 1000, getName(), currentStepVoltage,
                distance.get());
            minVoltage = currentStepVoltage;
            return;
        }

        // Have we tried this voltage long enough?
        if (now >= stepStartTime + STEP_DURATION_MS) {
            stepStartTime += STEP_DURATION_MS;
            currentStepVoltage += STEP_SIZE_VOLTS;

            System.out.printf("%3d %s Stepping voltage V=%5.3f dist=%5.3f\n",
                now % 1000, getName(), currentStepVoltage,
                distance.get());
        }

        drive.accept(currentStepVoltage);
        return;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%s is finished, minVoltage=%5.3f\n", getName(), minVoltage);
        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = minVoltage > 0.0;

        return isFinished;
    }
}
