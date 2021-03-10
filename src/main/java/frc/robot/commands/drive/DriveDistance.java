package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveDistance extends CommandBase {
    private static final double SPEED = 1.0;

    private RomiDrivetrain drivetrain;
    private double inches;

    public DriveDistance(double inches, RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.inches = inches;

        addRequirements(drivetrain);
        setName("DriveDistance(" + inches + ")");
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
        drivetrain.arcadeDrive(Math.signum(inches) * SPEED, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished = Math.abs(drivetrain.getAvgDistanceInches()) > Math.abs(inches);

        System.out.printf("%s is %sfinished, left=%3.1f right=%3.1f\n",
            getName(), isFinished ? "" : "not ",
            drivetrain.getLeftDistanceInches(), drivetrain.getRightDistanceInches());

        return isFinished;
    }
}
