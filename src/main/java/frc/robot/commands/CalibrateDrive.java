package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class CalibrateDrive extends CommandBase {
    private RomiDrivetrain drivetrain;

    /**
     * Creates a command that analyzes the distance driven and suggests adjustments to drive calibration.
     * @param drivetrain
     */
    public CalibrateDrive(RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double left = drivetrain.getLeftDistanceInches();
        double right = drivetrain.getRightDistanceInches();
        double avg = drivetrain.getAvgDistanceInches();

        double leftAdjust = avg / left;
        double rightAdjust = avg / right;

        System.out.printf("%s is finished, left=%5.3f, right=%5.3f, avg=%5.3f, leftAdjust=%5.3f rightAdjust=%5.3f\n",
            getName(), left, right, avg, leftAdjust, rightAdjust);

        return true;
    }
}
