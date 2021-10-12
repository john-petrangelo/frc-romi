package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final int ARM_SERVO_CHANNEL = 3;

  private final Servo servo = new Servo(ARM_SERVO_CHANNEL);

  /** 
   * The servo's valid range is roughly +/- 75 degrees
   */
  public final double RANGE = 75.0;

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
   * @param degrees The new position expressed in degrees: zero = center, negative = CCW, positive = CW
   */
  public void setPosition(double degrees) {
    // Clamp the input degrees to the valid range.
    degrees = Math.max(-RANGE, degrees);
    degrees = Math.min(degrees, RANGE);

    // The new arm position is given in degrees, but the servo takes a range from 0 to 1.0.
    // Map the requested degrees into the correct servo position value.
    double position = 1.0 - (degrees + RANGE) / (2.0 * RANGE);

    // Tell the servo where to go.
    servo.setPosition(position);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(".position", () -> servo.getPosition(), null);
  }
}
