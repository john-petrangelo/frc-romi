package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.calibration.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.WriteMessage;
import frc.robot.commands.Pause;
import frc.robot.sensors.RomiGyro;
import frc.robot.GrayBlueController.Axes;
import frc.robot.GrayBlueController.Buttons;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final RomiDrivetrain drivetrain = new RomiDrivetrain();
  private final RomiGyro gyro = new RomiGyro();

  private final Joystick controller = new Joystick(0);
  // private final JoystickButton buttonA = new JoystickButton(controller, Buttons.A.value);
  // private final JoystickButton buttonB = new JoystickButton(controller, Buttons.B.value);
  private final JoystickButton buttonX = new JoystickButton(controller, Buttons.X.value);
  private final JoystickButton buttonY = new JoystickButton(controller, Buttons.Y.value);
  private final JoystickButton buttonBack = new JoystickButton(controller, Buttons.Back.value);
  private final JoystickButton buttonStart = new JoystickButton(controller, Buttons.Start.value);
  private final JoystickButton povUp = new JoystickButton(controller, Buttons.POVup.value);
  private final JoystickButton povDown = new JoystickButton(controller, Buttons.POVdown.value);
  private final JoystickButton povLeft = new JoystickButton(controller, Buttons.POVleft.value);
  private final JoystickButton povRight = new JoystickButton(controller, Buttons.POVright.value);
  private final JoystickButton leftBumper = new JoystickButton(controller, Buttons.BumperLeft.value);
  private final JoystickButton rightBumper = new JoystickButton(controller, Buttons.BumperRight.value);

  public RobotContainer() {
    // The default command is run when no other commands are active.
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
        () -> -controller.getRawAxis(Axes.LeftY.value),
        () ->  controller.getRawAxis(Axes.RightX.value)
    ));

    configureButtonBindings();
  }
 
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    povUp.whenPressed(new DriveDistance(36.0, drivetrain));
    povDown.whenPressed(new DriveDistance(-36.0, drivetrain));
    // povUp.whenPressed(new DriveForwardWithGyro(36.0, drivetrain, gyro));
    // povDown.whenPressed(new DriveForwardWithGyro(-36.0, drivetrain, gyro));
    // povUp.whenPressed(new DriveTrapezoid(36.0, drivetrain));
    // povDown.whenPressed(new DriveTrapezoid(-36.0, drivetrain));

    povLeft.whenPressed(new TurnWithGyro(-73, drivetrain, gyro));
    povRight.whenPressed(new TurnWithGyro(73, drivetrain, gyro));
    // povLeft.whenPressed(new TurnToAngleWithPID(-90, drivetrain, gyro));
    // povRight.whenPressed(new TurnToAngleWithPID(90, drivetrain, gyro));
    // povLeft.whenPressed(new TurnTrapezoid(-90.0, drivetrain, gyro));
    // povRight.whenPressed(new TurnTrapezoid(90.0, drivetrain, gyro));

    leftBumper.whenPressed(()  -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.FF));
    rightBumper.whenPressed(() -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.NT_VOLTS));
    buttonBack.whenPressed(()  -> gyro.reset());
    buttonStart.whenPressed(() -> drivetrain.publishParams());

    buttonX.whenPressed(new FindMinVoltage(drivetrain));
    buttonY.whenPressed(
      new SequentialCommandGroup(
        new WriteMessage("Starting calibration sequence, will drive 12 inches then analyze the results"),
        new DriveDistance(12, drivetrain),
        new DriveStop(drivetrain),
        new CalibrateDrive(drivetrain),
        new Pause(0.3),
        new WriteMessage("Settled for 300ms"),
        new CalibrateDrive(drivetrain),
        new WriteMessage("Calibration sequence complete")
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command cmds = new SequentialCommandGroup(
      new DriveDistance(12, drivetrain),
      new TurnWithGyro(90, drivetrain, gyro),
      new DriveDistance(12, drivetrain),
      new TurnWithGyro(-45, drivetrain, gyro),
      new DriveDistance(12 * Math.sqrt(2), drivetrain),
      new TurnWithGyro(45+90, drivetrain, gyro),
      new DriveDistance(24, drivetrain),
      new TurnWithGyro(90, drivetrain, gyro),
      new DriveDistance(24, drivetrain),
      new TurnWithGyro(90, drivetrain, gyro)
    );

    return cmds;
  }

  public Command getTestCommand() {
    return new DriveDistance(12.0, drivetrain);
  }
}
