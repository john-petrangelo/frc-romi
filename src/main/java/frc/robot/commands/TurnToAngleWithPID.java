package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * A command that will turn the robot to the specified angle using a PID controller.
 */
public class TurnToAngleWithPID extends PIDCommand {
  private static final double TURN_TOLERANCE_DEG = 1.0;
  private static final double TURN_RATE_TOLERANCE_DEG_PER_SEC = 5.0;
  private static final double kP = 0.0072;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private final RomiGyro gyro;

  /**
   * Turns the robot the specified number of degrees.
   *
   * @param targetAngleDegrees The number of degrees to turn. Positive is clockwise.
   * @param drive The RomiDrivetrain subsystem to use for driving
   * @param drive The RomiGryo subsystem to use for heading
   */
  public TurnToAngleWithPID(double angle, RomiDrivetrain drive, RomiGyro gyro) {
    super(
        new PIDController(kP, kI, kD),
        () -> -gyro.getAngleYaw(),              // Closed loop on heading
        angle,                                  // Set reference to target
        output -> drive.arcadeDrive(0, output), // Pipe output to turn robot
        drive, gyro);                           // Required subsystems

    // Remember the gyro for later use.
    this.gyro = gyro;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);

    // Set the controller tolerances
    getController().setTolerance(TURN_TOLERANCE_DEG, TURN_RATE_TOLERANCE_DEG_PER_SEC);

    setName("TurnToAngleWithPID(" + angle + ")");
  }

  @Override
  public void initialize() {
    System.out.printf("%s initalize\n", getName());
    gyro.reset();
  }

  @Override
  public boolean isFinished() {
    boolean isFinished = getController().atSetpoint();
    System.out.printf("%s is %sfinished, degrees=%3.1f rate=%3.1f\n",
      getName(), isFinished ? "" : "not ", gyro.getAngleYaw(), gyro.getRateYaw());
    return isFinished;
  }
}
