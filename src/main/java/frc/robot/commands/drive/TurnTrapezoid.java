package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RomiMap;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnTrapezoid extends ProfiledPIDCommand {
    private static final Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(30, 30);
    private static final double TURN_TOLERANCE_DEG = 1.0;
    private static final double TURN_RATE_TOLERANCE_DEG_PER_SEC = 5.0;
    private static final double kP = 0.072;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
          
    private final RomiGyro gyro;

    public TurnTrapezoid(double angle, RomiDrivetrain drivetrain, RomiGyro gyro) {
        super(
            new ProfiledPIDController(kP, kI, kD, CONSTRAINTS),
            () -> -gyro.getAngleYaw(),
            angle,
            (output, setpoint) -> drivetrain.arcadeDrive(0.0, output / RomiMap.MAX_SPEED),
            drivetrain, gyro);

        this.gyro = gyro;

        // Set the controller to be continuous (because it is an angle controller)
        SendableRegistry.setName(getController(), "TurnTrapezoid controller");
        getController().enableContinuousInput(-180, 180);

        // Set the controller tolerances
        getController().setTolerance(TURN_TOLERANCE_DEG, TURN_RATE_TOLERANCE_DEG_PER_SEC);

        addRequirements(drivetrain);
        setName("TurnTrapezoid(" + angle + ")");
    }

    @Override
    public void initialize() {
        System.out.printf("%s initialize", getName());
        super.initialize();
        gyro.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished = getController().atGoal();

        System.out.printf("%s is %sfinished, degrees=%3.1f rate=%3.1f\n",
            getName(), isFinished ? "" : "not ", gyro.getAngleYaw(), gyro.getRateYaw());
  
        return isFinished;
    }
}
