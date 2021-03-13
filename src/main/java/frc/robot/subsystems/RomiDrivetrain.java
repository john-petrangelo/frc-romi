package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SlewRateLimiter;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.speedcontrollers.FeedforwardSpeedController;

public class RomiDrivetrain extends SubsystemBase {
  private static final double COUNTS_PER_REVOLUTION = 1440.0;
  private static final double WHEEL_DIAMETER_INCHES = 2.75;
  private static final double INCHES_PER_TICK = Math.PI * WHEEL_DIAMETER_INCHES / COUNTS_PER_REVOLUTION;

  public static final double MAX_SPEED = 20.0;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Limit the amount of change allowed per iteration for each sensor.
  private SlewRateLimiter leftDriveSpeed = new SlewRateLimiter(100);
  private SlewRateLimiter rightDriveSpeed = new SlewRateLimiter(100);

  private final FeedforwardSpeedController leftFFController;
  private final FeedforwardSpeedController rightFFController;

  // Set up the differential drive controller.
  private final DifferentialDrive diffDrive;

  private static class Characteristics {
    double kSLeftFwd;
    double kVLeftFwd;

    double kSLeftBack;
    double kVLeftBack;

    double kSRightFwd;
    double kVRightFwd;

    double kSRightBack;
    double kVRightBack;

    Characteristics(double kSLeftFwd, double kVLeftFwd,
                    double kSLeftBack, double kVLeftBack,
                    double kSRightFwd, double kVRightFwd,
                    double kSRightBack, double kVRightBack) {
      this.kSLeftFwd = kSLeftFwd;
      this.kVLeftFwd = kVLeftFwd;

      this.kSLeftBack = kSLeftBack;
      this.kVLeftBack = kVLeftBack;
  
      this.kSRightFwd = kSRightFwd;
      this.kVRightFwd = kVRightFwd;

      this.kSRightBack = kSRightBack;
      this.kVRightBack = kVRightBack;
    }

    @Override
    public String toString() {
        return "Characteristics Left=[kSFwd=" + kSLeftFwd
          + ", kVFwd=" + kVLeftFwd 
          + ", kSBack=" + kSLeftBack
          + ", kVBack=" + kVLeftBack
          + "], Right=[kSFwd=" + kSRightFwd 
          + ", kVFwd=" + kVRightFwd
          + ", kSBack=" + kSRightBack
          + ", kVBack=" + kVRightBack + "]";
    }
  }
  
  private final static Characteristics myData = new Characteristics(
      0.4687, 0.2381,
      0.2993, 0.2685,
      0.5561, 0.2666,
      0.7771, 0.2545);
      
  private Characteristics data = myData;

  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain() {
    super();
    
    leftEncoder.setDistancePerPulse(INCHES_PER_TICK);
    rightEncoder.setDistancePerPulse(INCHES_PER_TICK);

    // Create the speed controllers used for the various test modes.
    leftFFController = new FeedforwardSpeedController("L", leftMotor,
      data.kSLeftFwd, data.kVLeftFwd, data.kSLeftBack, data.kVLeftBack);
    rightFFController = new FeedforwardSpeedController("R", rightMotor,
      data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
      // data.kSRightFwd, data.kVRightFwd, data.kSRightBack, data.kVRightBack);

    // Set up the differential drive controllers for the various test modes.
    diffDrive = new DifferentialDrive(leftFFController, rightFFController);
    diffDrive.setMaxOutput(MAX_SPEED);
    diffDrive.setDeadband(0.12);

    resetEncoders();
  }

  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("ArcadeSpeed", speed);
    SmartDashboard.putNumber("ArcadeRotation", rotation);

    diffDrive.arcadeDrive(speed, rotation, false);
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
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistanceInches() {
    return leftEncoder.getDistance();
  }

  public double getRightDistanceInches() {
    return rightEncoder.getDistance();
  }

  public double getMinDistanceInches() {
    return Math.min(leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public double getMaxDistanceInches() {
    return Math.max(leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public double getAvgDistanceInches() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance() / 2);
  }

  /**
   * Returns the left motor rate in inches per second.
   */
  public double getLeftRate() {
    return leftEncoder.getRate();
  }

  /**
   * Returns the right motor rate in inches per second.
   */
  public double getRightRate() {
    return rightEncoder.getRate();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(".leftSpeed", () -> getLeftRate(), null);
    builder.addDoubleProperty(".rightSpeed", () -> getRightRate(), null);

    builder.addDoubleProperty(".leftSpeedSmoothed", () -> leftDriveSpeed.calculate(getLeftRate()), null);
    builder.addDoubleProperty(".rightSpeedSmoothed", () -> rightDriveSpeed.calculate(getRightRate()), null);

    builder.addDoubleProperty(".leftDistance", () -> getLeftDistanceInches(), null);
    builder.addDoubleProperty(".rightDistance", () -> getRightDistanceInches(), null);
  }
}
