package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPositionCmd extends CommandBase {
    private final Arm arm;
    private final double position;

    public ArmPositionCmd(double position, Arm arm) {
        this.position = position;
        this.arm = arm;

        addRequirements(arm);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setPosition(position);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
