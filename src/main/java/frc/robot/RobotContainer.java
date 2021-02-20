package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveBackward;
import frc.robot.commands.DriveForward;
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

  private final Joystick controller = new Joystick(0);
  private final JoystickButton buttonA = new JoystickButton(controller, Buttons.A.value);
  private final JoystickButton buttonB = new JoystickButton(controller, Buttons.B.value);
  private final JoystickButton buttonX = new JoystickButton(controller, Buttons.X.value);
  private final JoystickButton buttonY = new JoystickButton(controller, Buttons.Y.value);
  private final JoystickButton buttonStart = new JoystickButton(controller, Buttons.Start.value);
  private final JoystickButton povUp = new JoystickButton(controller, Buttons.POVup.value);
  private final JoystickButton povDown = new JoystickButton(controller, Buttons.POVdown.value);
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
    povUp.whenPressed(new DriveForward(drivetrain, 12.0));
    povDown.whenPressed(new DriveBackward(drivetrain, 12.0));

    buttonA.whenPressed(()     -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.PIDF));
    buttonB.whenPressed(()     -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.PIDF_PZ));
    buttonX.whenPressed(()     -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.FF));
    buttonY.whenPressed(()     -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.RAW));
    rightBumper.whenPressed(() -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.NT_VOLTS));
    buttonStart.whenPressed(() -> drivetrain.publishParams());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new DriveForward(drivetrain, 24.0);
  }

  public Command getTestCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveBackward(drivetrain, 24.0);
  }
}
