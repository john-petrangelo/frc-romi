package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPositionCmd extends CommandBase {
    private final Arm arm;
    private final double degrees;

    public ArmPositionCmd(double degrees, Arm arm) {
        this.degrees = degrees;
        this.arm = arm;

        addRequirements(arm);
    }

    // Called once when the command is scheduled.
    @Override
    public void initialize() {
        arm.setPosition(degrees);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
