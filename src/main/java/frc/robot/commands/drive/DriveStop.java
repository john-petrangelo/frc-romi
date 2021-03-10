package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveStop extends CommandBase {
    private RomiDrivetrain drivetrain;

    public DriveStop(RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0.0, 0.0);
        System.out.printf("%s stopped\n", getName());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
