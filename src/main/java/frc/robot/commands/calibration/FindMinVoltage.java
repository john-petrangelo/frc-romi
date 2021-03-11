package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class FindMinVoltage extends CommandBase {
    private static final long STEP_DURATION_MS = 100;
    private static final double STEP_SIZE_VOLTS = 0.05;

    private RomiDrivetrain drivetrain;
    private long stepStartTime;
    private double currentStepVoltage;
    private double leftMinVoltage = -1.0;
    private double rightMinVoltage = -1.0;

    /**
     * Creates a command finds the minimum voltage needed to move the robot.
     */
    public FindMinVoltage(RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        stepStartTime = System.currentTimeMillis();
        currentStepVoltage = STEP_SIZE_VOLTS;
        leftMinVoltage = -1.0;
        rightMinVoltage = -1.0;

        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        // If it's too soon, don't bother checking.
        long now = System.currentTimeMillis();
        System.out.printf("%3d %s V=%5.3f L=%5.3f R=%5.3f\n",
            now % 1000, getName(), currentStepVoltage,
            drivetrain.getLeftDistanceInches(), drivetrain.getRightDistanceInches());

        if (leftMinVoltage < 0.0 && drivetrain.getLeftDistanceInches() > 0.0) {
            leftMinVoltage = currentStepVoltage;
        }

        if (rightMinVoltage < 0.0 && drivetrain.getRightDistanceInches() > 0.0) {
            rightMinVoltage = currentStepVoltage;
        }

        if (now >= stepStartTime + STEP_DURATION_MS) {
            stepStartTime += STEP_DURATION_MS;
            currentStepVoltage += STEP_SIZE_VOLTS;
        }

        drivetrain.voltDrive(currentStepVoltage, 0.0);
        return;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%s is finished, leftMinVoltage=%5.3f, rightMinVoltage=%5.3f\n",
                getName(), leftMinVoltage, rightMinVoltage);
        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = leftMinVoltage > 0.0 && rightMinVoltage > 0.0;

        return isFinished;
    }
}
