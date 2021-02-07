package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveBackward extends CommandBase {
    private RomiDrivetrain drivetrain;
    private double distance;

    public DriveBackward(RomiDrivetrain drivetrain, double distance) {
        System.out.println("DriveBackward ctor");
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
        setName("DriveBackward " + distance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("DriveBackward.init()");
        drivetrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("DriveBackward.execute()");
        drivetrain.arcadeDrive(-0.5, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        System.out.print("DriveBackward.isFinished()");
        System.out.print(" left=" + drivetrain.getLeftDistanceInch());
        System.out.print(" distance=" + distance);
        System.out.println(" right=" + drivetrain.getRightDistanceInch());
        return drivetrain.getRightDistanceInch() < -distance &&
               drivetrain.getLeftDistanceInch()  < -distance;
    }
}
