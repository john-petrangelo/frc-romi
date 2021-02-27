package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SlewRateLimiter;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.speedcontrollers.FeedforwardSpeedController;
import frc.robot.speedcontrollers.FixedVoltsSpeedController;
import frc.robot.speedcontrollers.PIDFSpeedController;
import frc.robot.speedcontrollers.PIDSpeedController;

public class RomiDrivetrain extends SubsystemBase {
  private static final double COUNTS_PER_REVOLUTION = 1440.0;
  private static final double WHEEL_DIAMETER_INCHES = 2.75;
  private static final double MAX_SPEED = 20.0;

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

  private final PIDSpeedController leftPIDController;
  private final PIDSpeedController rightPIDController;

  private final PIDFSpeedController leftPIDFController;
  private final PIDFSpeedController rightPIDFController;

  private final FixedVoltsSpeedController leftNTVoltsController;
  private final FixedVoltsSpeedController rightNTVoltsController;

  // Set up the differential drive controllers.
  private final DifferentialDrive diffDriveRaw;
  private final DifferentialDrive diffDriveFF;
  private final DifferentialDrive diffDrivePID;
  private final DifferentialDrive diffDrivePIDF;
  private final DifferentialDrive diffDriveNTVolts;
  private DifferentialDrive activeDiffDrive;

  public enum DiffDriveMode {
    RAW, FF, PIDF_PZ, PIDF, NT_VOLTS
  }

  private static class Characteristics {
    double kSLeftFwd;
    double kVRightFwd;
    double kVLeftFwd;
    double kPLeftFwd;
    double kSLeftBack;
    double kVLeftBack;
    double kPLeftBack;

    double kSRightFwd;
    double kPRightFwd;
    double kSRightBack;
    double kVRightBack;
    double kPRightBack;

    Characteristics(double kSLeftFwd, double kVLeftFwd, double kPLeftFwd,
                    double kSLeftBack, double kVLeftBack, double kPLeftBack,
                    double kSRightFwd, double kVRightFwd, double kPRightFwd,
                    double kSRightBack, double kVRightBack, double kPRightBack) {
      this.kSLeftFwd = kSLeftFwd;
      this.kVLeftFwd = kVLeftFwd;
      this.kPLeftFwd = kPLeftFwd;
      this.kSLeftBack = kSLeftBack;
      this.kVLeftBack = kVLeftBack;
      this.kPLeftBack = kPLeftBack;
  
      this.kSRightFwd = kSRightFwd;
      this.kVRightFwd = kVRightFwd;
      this.kPRightFwd = kPRightFwd;
      this.kSRightBack = kSRightBack;
      this.kVRightBack = kVRightBack;
      this.kPRightBack = kPRightBack;
    }
  }
  
  private final static Characteristics myData = new Characteristics(
      0.4687, 0.2365, 0.0,
      0.2993, 0.2701, 0.0,
      0.5561, 0.2684, 0.0,
      0.7771, 0.2531, 0.0);
      
  // private Characteristics data = dataJan17Groomed;
  private Characteristics data = myData;

  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain() {
    leftEncoder.setDistancePerPulse(ticksToInches(1));
    rightEncoder.setDistancePerPulse(ticksToInches(1));

    // Create the speed controllers used for the various test modes.
    leftFFController = new FeedforwardSpeedController("L", leftMotor,
      data.kSLeftFwd, data.kVLeftFwd, data.kSLeftBack, data.kVLeftBack);
    rightFFController = new FeedforwardSpeedController("R", rightMotor,
      data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
      // data.kSRightFwd, data.kVRightFwd, data.kSRightBack, data.kVRightBack);

    leftPIDController = new PIDSpeedController(leftMotor, leftEncoder::getRate, data.kPLeftFwd, 0.0, 0.0);
    rightPIDController = new PIDSpeedController(rightMotor, rightEncoder::getRate, data.kPRightFwd, 0.0, 0.0);
  
    leftPIDFController = new PIDFSpeedController("L", leftMotor, leftEncoder::getRate,
      data.kSLeftFwd,  data.kVLeftFwd,  data.kPLeftFwd,
      data.kSLeftBack, data.kVLeftBack, data.kPLeftBack);
    rightPIDFController = new PIDFSpeedController("R", rightMotor, rightEncoder::getRate,
      data.kSRightBack, data.kVRightBack, data.kPRightBack,
      data.kSRightFwd,  data.kVRightFwd,  data.kPRightFwd);

    leftNTVoltsController = new FixedVoltsSpeedController("L", leftMotor, leftEncoder::getRate, 4.0);
    rightNTVoltsController = new FixedVoltsSpeedController("R", rightMotor, rightEncoder::getRate, 4.0);

    // Set up the differential drive controllers for the various test modes.
    diffDriveRaw = new DifferentialDrive(leftMotor, rightMotor);
    diffDriveFF = new DifferentialDrive(leftFFController, rightFFController);
    diffDrivePID = new DifferentialDrive(leftPIDController, rightPIDController);
    diffDrivePIDF = new DifferentialDrive(leftPIDFController, rightPIDFController);
    diffDriveNTVolts = new DifferentialDrive(leftNTVoltsController, rightNTVoltsController);

    diffDriveFF.setMaxOutput(MAX_SPEED);
    diffDrivePID.setMaxOutput(MAX_SPEED);
    diffDrivePIDF.setMaxOutput(MAX_SPEED);
    diffDriveNTVolts.setMaxOutput(MAX_SPEED);

    diffDriveFF.setDeadband(0.12);
    diffDrivePID.setDeadband(0.12);
    diffDrivePIDF.setDeadband(0.12);
    diffDriveNTVolts.setDeadband(0.12);

    // Default to "raw" mode.
    setDiffDriveMode(DiffDriveMode.RAW);

    setupNetworkTablesListeners();
  }

  public void setDiffDriveMode(DiffDriveMode mode) {
    final String driveModeKey = "Romi-O/DriveMode";

    System.out.println("New drive mode: " + mode.toString());
    switch (mode) {
      case RAW:
        activeDiffDrive = diffDriveRaw;
        SmartDashboard.putString(driveModeKey, "Raw");
        break;
      case FF:
        activeDiffDrive = diffDriveFF;
        leftFFController.setParameters (data.kSLeftFwd,   data.kVLeftFwd,   data.kSLeftBack, data.kVLeftBack);
        rightFFController.setParameters(data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
        SmartDashboard.putString(driveModeKey, "FF");
        break;
      case PIDF_PZ:
        activeDiffDrive = diffDrivePIDF;
        leftPIDFController.setParameters( data.kSLeftFwd,   data.kVLeftFwd,   0.0,
                                          data.kSLeftBack,  data.kVLeftBack,  0.0);
        rightPIDFController.setParameters(data.kSRightBack, data.kVRightBack,0.0,
                                          data.kSRightFwd,  data.kVRightFwd, 0.0);
        SmartDashboard.putString(driveModeKey, "FF + PID(kP=0)");
        break;
        case PIDF:
        activeDiffDrive = diffDrivePIDF;
        leftPIDFController.setParameters( data.kSLeftFwd,   data.kVLeftFwd,   data.kPLeftFwd,
                                          data.kSLeftBack,  data.kVLeftBack,  data.kPLeftBack);
        rightPIDFController.setParameters(data.kSRightBack, data.kVRightBack,data.kPRightBack,
                                          data.kSRightFwd,  data.kVRightFwd, data.kPRightFwd);
        SmartDashboard.putString(driveModeKey, "FF + PID");
        break;
      case NT_VOLTS:
        activeDiffDrive = diffDriveNTVolts;
        double volts = SmartDashboard.getNumber("SetVolts", 0.0);
        leftNTVoltsController.setParameters(volts);
        rightNTVoltsController.setParameters(volts);
        SmartDashboard.putString(driveModeKey, "NT Volts");
        break;
    }
    
    resetEncoders();
  }

  private void setupNetworkTablesListeners() {
    NetworkTableInstance nti = NetworkTableInstance.getDefault();
    NetworkTable ntTable = nti.getTable("SmartDashboard/Romi-O");

    ntTable.addEntryListener((table, key, entry, value, flags) -> {
      System.out.println("Characterization data changed, " + key + ": " + value.getValue());
      if (key.startsWith("drive-")) {
        switch (key) {
          case "drive-left-fwd-kS":
            data.kSLeftFwd = value.getDouble();
            break;
          case "drive-left-fwd-kV":
            data.kVLeftFwd = value.getDouble();
            break;
          case "drive-left-fwd-kP":
            data.kPLeftFwd = value.getDouble();
            break;
          case "drive-left-back-kS":
            data.kSLeftBack = value.getDouble();
            break;
          case "drive-left-back-kV":
            data.kVLeftBack = value.getDouble();
            break;
          case "drive-left-back-kP":
            data.kPLeftBack = value.getDouble();
            break;
          case "drive-right-fwd-kS":
            data.kSRightFwd = value.getDouble();
            break;
          case "drive-right-fwd-kV":
            data.kVRightFwd = value.getDouble();
            break;
          case "drive-right-fwd-kP":
            data.kPRightFwd = value.getDouble();
            break;
          case "drive-right-back-kS":
            data.kSRightBack = value.getDouble();
            break;
          case "drive-right-back-kV":
            data.kVRightBack = value.getDouble();
            break;
          case "drive-right-back-kP":
            data.kPRightBack = value.getDouble();
            break;
        }
      }
    }, EntryListenerFlags.kUpdate);
  }

  public void publishParams() {
    String prefix = "Romi-O/";
    SmartDashboard.putNumber(prefix + "drive-left-fwd-kS", data.kSLeftFwd);
    SmartDashboard.putNumber(prefix + "drive-left-fwd-kV", data.kVLeftFwd);
    SmartDashboard.putNumber(prefix + "drive-left-fwd-kP", data.kPLeftFwd);

    SmartDashboard.putNumber(prefix + "drive-left-back-kS", data.kSLeftBack);
    SmartDashboard.putNumber(prefix + "drive-left-back-kV", data.kVLeftBack);
    SmartDashboard.putNumber(prefix + "drive-left-back-kP", data.kPLeftBack);

    SmartDashboard.putNumber(prefix + "drive-right-fwd-kS", data.kSRightFwd);
    SmartDashboard.putNumber(prefix + "drive-right-fwd-kV", data.kVRightFwd);
    SmartDashboard.putNumber(prefix + "drive-right-fwd-kP", data.kPRightFwd);

    SmartDashboard.putNumber(prefix + "drive-right-back-kS", data.kSRightBack);
    SmartDashboard.putNumber(prefix + "drive-right-back-kV", data.kVRightBack);
    SmartDashboard.putNumber(prefix + "drive-right-back-kP", data.kPRightBack);
  }

  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("ArcadeSpeed", speed);
    SmartDashboard.putNumber("ArcadeRotation", rotation);

    activeDiffDrive.arcadeDrive(speed, rotation, false);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return ticksToInches(leftEncoder.get());
  }

  public double getRightDistanceInch() {
    return ticksToInches(rightEncoder.get());
  }

  /**
   * Returns the left motor rate in inches.
   */
  public double getLeftRate() {
    return leftEncoder.getRate();
  }

  /**
   * Returns the right motor rate in inches.
   */
  public double getRightRate() {
    return rightEncoder.getRate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftSpeed", getLeftRate());
    SmartDashboard.putNumber("RightSpeed", getRightRate());
 
    SmartDashboard.putNumber("LeftSpeed-Smoothed", leftDriveSpeed.calculate(getLeftRate()));
    SmartDashboard.putNumber("RightSpeed=Smoothed", rightDriveSpeed.calculate(getRightRate()));
 
    SmartDashboard.putNumber("LeftDistance", getLeftDistanceInch());
    SmartDashboard.putNumber("RightDistance", getRightDistanceInch());

    // Only the "active" differential drive feeds its speed controllers.
    // Explicitly feed all of them so nobody starves.
    diffDriveRaw.feed();
    diffDriveFF.feed();
    diffDrivePID.feed();
    diffDrivePIDF.feed();
    diffDriveNTVolts.feed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Convert wheel encoder ticks to inches based on wheel physical dimensions.
  private double ticksToInches(double ticks) {
    return Math.PI * WHEEL_DIAMETER_INCHES * (ticks / COUNTS_PER_REVOLUTION);
  }
}
