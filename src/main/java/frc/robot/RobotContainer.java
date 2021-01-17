/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain drivetrain = new RomiDrivetrain();

  // private final OnBoardIO onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channel 0.
  private final Joystick controller = new Joystick(0);
  private final JoystickButton buttonA = new JoystickButton(controller, XboxController.Button.kA.value);
  private final JoystickButton buttonB = new JoystickButton(controller, XboxController.Button.kB.value);
  private final JoystickButton buttonX = new JoystickButton(controller, XboxController.Button.kX.value);
  private final JoystickButton buttonY = new JoystickButton(controller, XboxController.Button.kY.value);
  private final JoystickButton povUp = new JoystickButton(controller, 12);

  // private final JoystickButton povDown = new JoystickButton(controller, 13);

  private final ExampleCommand autoCommand = new ExampleCommand(drivetrain);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }
 
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain,
        () -> -controller.getRawAxis(1),
        () ->  controller.getRawAxis(3)
    ));

    povUp.whenPressed(new DriveForward(drivetrain, 12.0));

    buttonA.whenPressed(() -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.PIDF));
    buttonB.whenPressed(() -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.PID));
    buttonX.whenPressed(() -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.FF));
    buttonY.whenPressed(() -> drivetrain.setDiffDriveMode(RomiDrivetrain.DiffDriveMode.RAW));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
