package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final int ARM_SERVO_CHANNEL = 3;

  private final Servo servo = new Servo(ARM_SERVO_CHANNEL);

  /**
   * Creates a new Arm subsystem.
   */
  public Arm() {
    super();
  }

  /**
   * Gets the current position of the arm servo.
   */
  public double getPosition() {
    return servo.getPosition();
  }

  /**
   * Sets the position where we want the arm servo to be.
   * 
   * @param position The new position. 0 = full clockwise, 1.0 = full counterclockwise, 0.5 = center.
   */
  public void setPosition(double position) {
    servo.setPosition(position);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(".position", () -> servo.getPosition(), null);
  }
}
