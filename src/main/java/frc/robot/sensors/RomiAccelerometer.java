package frc.robot.sensors;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiAccelerometer extends SubsystemBase {
  private final double PERIOD_SECS = 0.020; 

  // The accelerometer reads no-zero values at rest. Adjust these values to
  // compensate for this offset.
  private final double X_OFFSET = 0.0160; //0.0038;
  private final double Y_OFFSET = 0.0087;
  private final double Z_OFFSET = 0.0320;

  private final Accelerometer accel;

  // Speed accumulators.
  // NOTE: These speed values are not reliable. More work is necessary to identify down direction
  // and compensate knowing that we are not in fact free-falling.
  private double speedX = 0.0;
  private double speedY = 0.0;
  private double speedZ = 0.0;

  /** Create a new RomiGyro. */
  public RomiAccelerometer() {
    super();

    accel = new BuiltInAccelerometer();
  }

  // Gravitational acceleration on Earth's surface is 32.1741 ft/sec^2.
  private double gToInches(double g) {
    return g * 32.1741 * 12;
  }

  /**
   * Get the current acceleration along the x axis in inches/sec^2.
   */
  public double getX() {
    if (accel != null) {
      return gToInches(accel.getX() - X_OFFSET);
    }

    return 0.0;
  }

  /**
   * Get the current acceleration along the x axis in inches/sec^2.
   */
  public double getY() {
    if (accel != null) {
      return gToInches(accel.getY() - Y_OFFSET);
    }

    return 0.0;
  }

  /**
   * Get the current acceleration along the x axis in inches/sec^2.
   */
  public double getZ() {
    if (accel != null) {
      // Note: we subtract 1g to compensate for gravity. However, this assumes that
      // we are exactly upright and gravity is entirely along the z-axis. More refinement
      // is necessary before this is useful.
      return gToInches(accel.getZ() - Z_OFFSET - 1.0);
    }

    return 0.0;
  }

  // Reset the speed accumulators.
  public void reset() {
    System.out.println("Reseting accelerometer");
    speedX = 0.0;
    speedY = 0.0;
    speedZ = 0.0;    
  }

  @Override
  public void periodic() {
    // Accumulate acclerations into speeds.
    speedX += getX() * PERIOD_SECS;
    speedY += getY() * PERIOD_SECS;
    speedZ += getZ() * PERIOD_SECS;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(".x", () -> getX(), null);
    builder.addDoubleProperty(".y", () -> getY(), null);
    builder.addDoubleProperty(".z", () -> getZ(), null);

    builder.addDoubleProperty(".speedX", () -> speedX, null);
    builder.addDoubleProperty(".speedY", () -> speedY, null);
    builder.addDoubleProperty(".speedZ", () -> speedZ, null);
  }
}
