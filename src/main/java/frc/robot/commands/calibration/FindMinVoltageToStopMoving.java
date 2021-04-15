package frc.robot.commands.calibration;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class FindMinVoltageToStopMoving extends CommandBase {
    public enum Direction {
        FORWARD, BACKWARD
    };

    private static final long STEP_DURATION_MS = 200;
    private static final double STEP_SIZE_VOLTS = 0.02;
    private static final double INITIAL_VOLTAGE = 1.5;

    private double initialLeftVoltage;
    private double initialRightVoltage;
    private RomiDrivetrain drivetrain;
    private long initTime;
    private long stepStartTime;
    private double currentLeftVoltage;
    private double currentRightVoltage;

    private Supplier<Double> leftRate;
    private Supplier<Double> rightRate;
    private Consumer<Double> leftDrive;
    private Consumer<Double> rightDrive;

    /**
     * Creates a command that finds the minimum voltage needed to keep the robot moving.
     * <p>
     * Starts with enough voltage for the robot to move and then slowly ramps
     * the voltage down until no more motion is detected.
     */
    public FindMinVoltageToStopMoving(Direction direction, RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.initialLeftVoltage = INITIAL_VOLTAGE;
        this.initialRightVoltage = INITIAL_VOLTAGE;

        switch(direction) {
        case FORWARD:
            leftRate  = () -> drivetrain.getLeftRate();
            rightRate = () -> drivetrain.getRightRate();
            leftDrive =  (volts) -> drivetrain.voltDriveLeft(volts);
            rightDrive = (volts) -> drivetrain.voltDriveRight(volts);
            break;
        case BACKWARD:
            leftRate =  () -> -drivetrain.getLeftRate();
            rightRate = () -> -drivetrain.getRightRate();
            leftDrive =  (volts) -> drivetrain.voltDriveLeft(-volts);
            rightDrive = (volts) -> drivetrain.voltDriveRight(-volts);
            break;
        default:
            throw new IllegalArgumentException("Invalid direction: " + direction);
        }

        setName(String.format("%s(%s)", "MinVoltsToStopMoving", direction.toString()));
    }

    @Override
    public void initialize() {
        super.initialize();
        initTime = System.currentTimeMillis();
        stepStartTime = initTime;
        currentLeftVoltage  = initialLeftVoltage;
        currentRightVoltage = initialRightVoltage;

        System.out.printf("%s Starting at LEFT(%5.3f) RIGHT(%5.3f) volts\n",
            getName(), currentLeftVoltage, currentRightVoltage);
    }

    @Override
    public void execute() {
        // If it's too soon, don't bother checking.
        final long now = System.currentTimeMillis();

        // Have we tried this voltage long enough?
        if (now >= stepStartTime + STEP_DURATION_MS) {
            stepStartTime += STEP_DURATION_MS;
            currentLeftVoltage  -= STEP_SIZE_VOLTS;
            currentRightVoltage -= STEP_SIZE_VOLTS;

            System.out.printf("%3d %s Stepping voltage LEFT(V=%5.3f rate=%5.3f) RIGHT(V=%5.3f rate=%5.3f)\n",
                now - initTime, getName(),
                currentLeftVoltage, leftRate.get(),
                currentRightVoltage, rightRate.get());
        }

        leftDrive.accept(currentLeftVoltage);
        rightDrive.accept(currentRightVoltage);
        return;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%s end LEFT(voltage=%5.3f rate=%5.3f) RIGHT(voltage=%5.3f rate=%5.3f)\n",
            getName(), currentLeftVoltage, leftRate.get(), currentRightVoltage, rightRate.get());
        SmartDashboard.putNumber("Calibration/" + getName() + "(LEFT)",  currentLeftVoltage);
        SmartDashboard.putNumber("Calibration/" + getName() + "(RIGHT)", currentRightVoltage);

        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        final double THRESHOLD = 0.1;
        return (Math.abs(leftRate.get()) < THRESHOLD && Math.abs(rightRate.get()) < THRESHOLD);
    }
}
