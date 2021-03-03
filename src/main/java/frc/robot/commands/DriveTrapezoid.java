package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveTrapezoid extends TrapezoidProfileCommand {
    private static final Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(20, 80);
          
    public DriveTrapezoid(double inches, RomiDrivetrain drivetrain) {
        super(
            new TrapezoidProfile(
                CONSTRAINTS,
                new TrapezoidProfile.State(inches, 0.0)),
            setpointState -> drivetrain.arcadeDrive(setpointState.velocity / RomiDrivetrain.MAX_SPEED, 0.0),
            drivetrain);

        addRequirements(drivetrain);
        setName("TurnTrapezoid(" + inches + ")");
    }

    @Override
    public void initialize() {
        System.out.printf("%s initialize", getName());
        super.initialize();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished = super.isFinished();

        System.out.printf("%s is %sfinished\n", getName(), isFinished ? "" : "not ");
  
        return isFinished;
    }
}
