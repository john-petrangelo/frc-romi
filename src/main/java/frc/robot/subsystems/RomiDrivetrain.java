package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.filters.ApacheCommonsKalman;
import frc.robot.filters.TrashCompactor;
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

  // The left and right wheel sensors.
  private final TrashCompactor leftWheelSensor;
  private final TrashCompactor rightWheelSensor;

  // Experimental Kalman filter
  private final ApacheCommonsKalman kalman;

  // Limit the amount of change allowed per iteration for each sensor.
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
      0.4687, 0.2381,  // LF
      0.2993, 0.2685,  // LB
      0.5561, 0.2666,  // RF
      0.7771, 0.2545); // RB

  private Characteristics data = myData;

  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain() {
    super();

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    leftWheelSensor = new TrashCompactor(new Encoder(4, 5));
    leftWheelSensor.getEncoder().setDistancePerPulse(INCHES_PER_TICK);
    rightWheelSensor = new TrashCompactor(new Encoder(6, 7));
    rightWheelSensor.getEncoder().setDistancePerPulse(INCHES_PER_TICK);

    // TODO Experimental Kalman filter
    kalman = new ApacheCommonsKalman(leftWheelSensor.getEncoder(), rightWheelSensor.getEncoder());

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
    leftWheelSensor.reset();
    rightWheelSensor.reset();
    kalman.reset();
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
    kalman.predict();
    kalman.update();

    SmartDashboard.putNumber("tc/distance(left)", getLeftDistanceInches());
    SmartDashboard.putNumber("tc/distance(right)", getRightDistanceInches());
    SmartDashboard.putNumber("tc/rate(left)", getLeftRate());
    SmartDashboard.putNumber("tc/rate(right)", getRightRate());

    SmartDashboard.putNumber("kf/distance(left)", kalman.getLeftPosition());
    SmartDashboard.putNumber("kf/distance(right)", kalman.getRightPosition());
    SmartDashboard.putNumber("kf/rate(left)", kalman.getLeftRate());
    SmartDashboard.putNumber("kf/rate(right)", kalman.getRightRate());

//    kalman.logP();
    logData();
  }

  private void logData() {
    System.out.printf("CSV,%d,%f,%f\n",
      System.currentTimeMillis(),
      getLeftRate(),
      getRightRate()
    );
  }
}
