package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RomiMap;
import frc.robot.filters.TrashCompactor;
import frc.robot.sensors.RomiGyro;
import frc.robot.speedcontrollers.FeedforwardSpeedController;

public class RomiDrivetrain extends SubsystemBase {
  // The Romi motors are modelled as Spark controllers using PWM outputs.
  private final Spark leftMotor = new Spark(RomiMap.LEFT_WHEEL_MOTOR_ID);
  private final Spark rightMotor = new Spark(RomiMap.RIGHT_WHEEL_MOTOR_ID);

  // The left and right wheel sensors.
  private final TrashCompactor leftWheelSensor;
  private final TrashCompactor rightWheelSensor;

  // Limit the amount of change allowed per iteration for each sensor.
  private final FeedforwardSpeedController leftFFController;
  private final FeedforwardSpeedController rightFFController;

  // Set up the differential drive controller.
  private final DifferentialDrive diffDrive;

  private final static DriveCharacteristics myData = new DriveCharacteristics(
      0.4687, 0.2381,  // LF
      0.2993, 0.2685,  // LB
      0.5561, 0.2666,  // RF
      0.7771, 0.2545); // RB

  private DriveCharacteristics data = myData;

  private RomiGyro gyro;

  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain(RomiGyro gyro) {
    super();

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    leftWheelSensor = new TrashCompactor(new Encoder(4, 5));
    leftWheelSensor.getEncoder().setDistancePerPulse(RomiMap.INCHES_PER_TICK);
    rightWheelSensor = new TrashCompactor(new Encoder(6, 7));
    rightWheelSensor.getEncoder().setDistancePerPulse(RomiMap.INCHES_PER_TICK);

    // Create the speed controllers used for the various test modes.
    leftFFController = new FeedforwardSpeedController("L", leftMotor,
      data.kSLeftFwd, data.kVLeftFwd, data.kSLeftBack, data.kVLeftBack);
    rightFFController = new FeedforwardSpeedController("R", rightMotor,
      data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
      // data.kSRightFwd, data.kVRightFwd, data.kSRightBack, data.kVRightBack);

    // Set up the differential drive controllers for the various test modes.
    diffDrive = new DifferentialDrive(leftFFController, rightFFController);
    diffDrive.setMaxOutput(RomiMap.MAX_SPEED);
    diffDrive.setDeadband(RomiMap.CONTROLS_DEADBAND);

    // TODO Depends on RomiGyro...
    this.gyro = gyro;

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

  /**
   * Clamps the input value to the range -1.0 to 1.0.
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   * <br><br>
   * NOTE: This method clamps and scales the inputs exactly the way the WPILib 
   * {@link edu.wpi.first.wpilibj.drive.DifferentialDrive DifferentialDrive} and
   * {@link edu.wpi.first.wpilibj.drive.RobotDriveBase RobotDriveBase} classes do.
   * 
   * @param value value to clamp and scale
   */
  protected double conditionControlInput(double value) {
    value = MathUtil.clamp(value, -1.0, 1.0);

    if (Math.abs(value) > RomiMap.CONTROLS_DEADBAND) {
      if (value > 0.0) {
        return (value - RomiMap.CONTROLS_DEADBAND) / (1.0 - RomiMap.CONTROLS_DEADBAND);
      } else {
        return (value + RomiMap.CONTROLS_DEADBAND) / (1.0 - RomiMap.CONTROLS_DEADBAND);
      }
    } else {
      return 0.0;
    }
  }

  public void voltDriveLeft(double voltage) {
    SmartDashboard.putNumber("VoltDrive-L", voltage);
    leftMotor.setVoltage(voltage);
    diffDrive.feed();
  }

  public void voltDriveRight(double voltage) {
    SmartDashboard.putNumber("VoltDrive-R", voltage);
    rightMotor.setVoltage(-voltage);
    diffDrive.feed();
  }

  public void voltDrive(double lVoltage, double rVoltage) {
    voltDriveLeft(lVoltage);
    voltDriveRight(rVoltage);
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

  public double getMinDistanceInches() {
    return Math.min(leftWheelSensor.getDistance(), rightWheelSensor.getDistance());
  }

  public double getMaxDistanceInches() {
    return Math.max(leftWheelSensor.getDistance(), rightWheelSensor.getDistance());
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
    leftWheelSensor.update();
    rightWheelSensor.update();

    SmartDashboard.putNumber("tc/distance(left)", getLeftDistanceInches());
    SmartDashboard.putNumber("tc/distance(right)", getRightDistanceInches());
    SmartDashboard.putNumber("tc/rate(left)", getLeftRate());
    SmartDashboard.putNumber("tc/rate(right)", getRightRate());
  }
}
