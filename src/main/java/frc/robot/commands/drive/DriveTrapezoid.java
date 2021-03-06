package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.RomiMap;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveTrapezoid extends TrapezoidProfileCommand {
    private static final Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(20, 80);
    private RomiDrivetrain drivetrain;
          
    public DriveTrapezoid(double inches, RomiDrivetrain drivetrain) {
        super(
            new TrapezoidProfile(
                CONSTRAINTS,
                new TrapezoidProfile.State(inches, 0.0)),
            setpointState -> drivetrain.arcadeDrive(setpointState.velocity / RomiMap.MAX_SPEED, 0.0),
            drivetrain);

        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
        setName("DriveTrapezoid(" + inches + ")");
    }

    @Override
    public void initialize() {
        super.initialize();

        System.out.printf("%s initialize", getName());
        drivetrain.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished = super.isFinished();

        System.out.printf("%s is %sfinished\n", getName(), isFinished ? "" : "not ");
  
        return isFinished;
    }
}
