package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RomiMap;
import frc.robot.filters.TrashCompactor;
import frc.robot.speedcontrollers.FeedforwardSpeedController;

public class RomiDrivetrain extends SubsystemBase {
  // The Romi motors are modelled as Spark controllers using PWM outputs.
  private final Spark leftMotor = new Spark(RomiMap.LEFT_WHEEL_MOTOR_ID);
  private final Spark rightMotor = new Spark(RomiMap.RIGHT_WHEEL_MOTOR_ID);

  // The Romi has onboard encoders that are hardcoded to use DIO pins 4/5
  // for the left side and 6/7 for the right side. Unfortunately the Romi
  // wheel sensors are noisy. We pass the raw sensor data through the
  // "TrashCompactor" filter to ignore bogus values.
  private final TrashCompactor leftWheelSensor = new TrashCompactor(new Encoder(4, 5));
  private final TrashCompactor rightWheelSensor = new TrashCompactor(new Encoder(6, 7));

  // Limit the amount of change allowed per iteration for each sensor.
  private final FeedforwardSpeedController leftFFController;
  private final FeedforwardSpeedController rightFFController;

  // Set up the differential drive controller.
  private final DifferentialDrive diffDrive;

  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain() {
    super();

    // Let the encoders know how far each sensor pulse is in inches.
    leftWheelSensor.getEncoder().setDistancePerPulse(RomiMap.INCHES_PER_TICK);
    rightWheelSensor.getEncoder().setDistancePerPulse(RomiMap.INCHES_PER_TICK);

    // Create the speed controllers used for the various test modes.
    leftFFController = new FeedforwardSpeedController("L", leftMotor,
      0.4687, 0.2381, // Left forward (kS, kV)
      0.2993, 0.2685  // Left backward (kS, kV)
    );

    rightFFController = new FeedforwardSpeedController("R", rightMotor,
      0.7771, 0.2545, // Right forward (kS, kV)
      0.5561, 0.2666  // Right backward (kS, kV)
    );

    // Set up the differential drive controllers for the various test modes.
    diffDrive = new DifferentialDrive(leftFFController, rightFFController);
    diffDrive.setMaxOutput(RomiMap.MAX_SPEED);
    diffDrive.setDeadband(RomiMap.CONTROLS_DEADBAND);

    // Set the initial position to 0 inches.
    resetEncoders();
  }

  /**
   * Arcade drive for Romi drivetrain.
   * 
   * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotation The robot's rotation rate [-1.0..1.0]. Clockwise is positive.
   */
 public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("ArcadeSpeed", speed);
    SmartDashboard.putNumber("ArcadeRotation", rotation);

    // Tell the motors what we want them to do.
    diffDrive.arcadeDrive(speed, rotation, false);
  }

  public void resetEncoders() {
    leftWheelSensor.reset();
    rightWheelSensor.reset();
  }

  public double getLeftDistanceInches() {
    return leftWheelSensor.getDistance();
  }

  public double getRightDistanceInches() {
    return rightWheelSensor.getDistance();
  }

  public double getAvgDistanceInches() {
    return (leftWheelSensor.getDistance() + rightWheelSensor.getDistance() / 2);
  }

  /**
   * Returns the left motor rate in inches per second.
   */
  public double getLeftRate() {
    return leftWheelSensor.getRate();
  }

  /**
   * Returns the right motor rate in inches per second.
   */
  public double getRightRate() {
    return rightWheelSensor.getRate();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(".leftSpeed", () -> getLeftRate(), null);
    builder.addDoubleProperty(".rightSpeed", () -> getRightRate(), null);

    builder.addDoubleProperty(".leftDistance", () -> getLeftDistanceInches(), null);
    builder.addDoubleProperty(".rightDistance", () -> getRightDistanceInches(), null);
  }

  @Override
  public void periodic() {
    // Update the TrashCompactor filter with the new sensor values.
    leftWheelSensor.update();
    rightWheelSensor.update();

    SmartDashboard.putNumber("tc/distance(left)", getLeftDistanceInches());
    SmartDashboard.putNumber("tc/distance(right)", getRightDistanceInches());
    SmartDashboard.putNumber("tc/rate(left)", getLeftRate());
    SmartDashboard.putNumber("tc/rate(right)", getRightRate());
  }
}
