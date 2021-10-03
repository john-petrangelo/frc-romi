package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.sensors.RomiGyro;
import frc.robot.GrayBlueController.Axes;
import frc.robot.GrayBlueController.Buttons;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.DriveDistanceCmd;
import frc.robot.commands.TurnDegreesCmd;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final RomiGyro gyro = new RomiGyro();
  private final RomiDrivetrain drivetrain = new RomiDrivetrain();

  // User interface objects
  private final Joystick controller = new Joystick(0);
  private final JoystickButton buttonA = new JoystickButton(controller, Buttons.A.value);
  private final JoystickButton buttonB = new JoystickButton(controller, Buttons.B.value);
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
    drivetrain.setDefaultCommand(new ArcadeDriveCmd(drivetrain,
        () -> -controller.getRawAxis(Axes.LeftY.value),
        () ->  controller.getRawAxis(Axes.RightX.value)
    ));

    configureButtonBindings();
  }
 
  /**
   * Connect the controller buttons to commands
   */
  private void configureButtonBindings() {
    povUp.whenPressed(new DriveDistanceCmd(36.0, drivetrain));
    povDown.whenPressed(new DriveDistanceCmd(-36.0, drivetrain));

    povLeft.whenPressed(new TurnDegreesCmd(-73, drivetrain, gyro));
    povRight.whenPressed(new TurnDegreesCmd(73, drivetrain, gyro));

    buttonBack.whenPressed(()  -> gyro.reset());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command cmds = new SequentialCommandGroup(
      new DriveDistanceCmd(12, drivetrain),
      new TurnDegreesCmd(90, drivetrain, gyro),
      new DriveDistanceCmd(12, drivetrain),
      new TurnDegreesCmd(-45, drivetrain, gyro),
      new DriveDistanceCmd(12 * Math.sqrt(2), drivetrain),
      new TurnDegreesCmd(45+90, drivetrain, gyro),
      new DriveDistanceCmd(24, drivetrain),
      new TurnDegreesCmd(90, drivetrain, gyro),
      new DriveDistanceCmd(24, drivetrain),
      new TurnDegreesCmd(90, drivetrain, gyro)
    );

    return cmds;
  }

  public Command getTestCommand() {
    return new DriveDistanceCmd(12.0, drivetrain);
  }
}
