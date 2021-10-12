package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.Arm;

/**
 * A command to make the arm always point in the same direction using the gyro.
 */
public class ArmPositionFollowGyroCmd extends CommandBase {
    private final Arm arm;
    private final RomiGyro gyro;

    public ArmPositionFollowGyroCmd(Arm arm, RomiGyro gyro) {
        this.arm = arm;
        this.gyro = gyro;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        gyro.reset();
    }

    @Override
    public void execute() {
        double angle = gyro.getAngleYaw();

        // Normalize the gyro angle to the range -180..180.
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360;
        }

        // Finally, tell the arm where to go.
        arm.setPosition(angle);
    }

    // This command will continue until it is cancelled.
    @Override
    public boolean isFinished() {
        return false;
    }
}
