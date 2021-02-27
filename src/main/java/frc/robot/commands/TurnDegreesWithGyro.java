package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnDegreesWithGyro extends CommandBase {
    private RomiDrivetrain drivetrain;
    private RomiGyro gyro;
    private double angle;

    public TurnDegreesWithGyro(RomiDrivetrain drivetrain, RomiGyro gyro, double angle) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.angle = angle;

        addRequirements(drivetrain);
        addRequirements(gyro);
        setName("TurnDegreesWithGyro(" + angle + ")");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.printf("%s initialize\n", getName());
        gyro.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.printf("%s execute\n", getName());
        drivetrain.arcadeDrive(0.0, Math.signum(angle) * 0.3);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished = Math.abs(gyro.getAngleYaw()) > Math.abs(angle);

        System.out.printf("%s is %sfinished, degrees=%3.1f\n",
            getName(), isFinished ? "" : "not ", gyro.getAngleYaw());

        return isFinished;
    }
}
