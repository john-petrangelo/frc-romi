package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveBackward extends CommandBase {
    private RomiDrivetrain drivetrain;
    private double distance;

    public DriveBackward(RomiDrivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
        setName("DriveBackward(" + distance + ")");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.printf("%s initialize\n", getName());
        drivetrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.printf("%s execute\n", getName());
        drivetrain.arcadeDrive(-0.5, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished =
            drivetrain.getRightDistanceInch() < -distance || drivetrain.getLeftDistanceInch()  < -distance;

        System.out.printf("%s is %sfinished, left=%3.1f right=%3.1f\n",
            getName(), isFinished ? "" : "not ",
            drivetrain.getLeftDistanceInch(), drivetrain.getRightDistanceInch());

        return isFinished;
    }
}
