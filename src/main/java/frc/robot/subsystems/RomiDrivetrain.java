package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SlewRateLimiter;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.speedcontrollers.FeedforwardSpeedController;
import frc.robot.speedcontrollers.FixedVoltsSpeedController;

public class RomiDrivetrain extends SubsystemBase {
  private static final double COUNTS_PER_REVOLUTION = 1440.0;
  private static final double WHEEL_DIAMETER_INCHES = 2.75;
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

  private final FixedVoltsSpeedController leftNTVoltsController;
  private final FixedVoltsSpeedController rightNTVoltsController;

  // Set up the differential drive controllers.
  private final DifferentialDrive diffDriveFF;
  private final DifferentialDrive diffDriveNTVolts;
  private DifferentialDrive activeDiffDrive;

  public enum DiffDriveMode {
    FF, NT_VOLTS
  }

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
    
    leftEncoder.setDistancePerPulse(ticksToInches(1));
    rightEncoder.setDistancePerPulse(ticksToInches(1));

    // Create the speed controllers used for the various test modes.
    leftFFController = new FeedforwardSpeedController("L", leftMotor,
      data.kSLeftFwd, data.kVLeftFwd, data.kSLeftBack, data.kVLeftBack);
    rightFFController = new FeedforwardSpeedController("R", rightMotor,
      data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
      // data.kSRightFwd, data.kVRightFwd, data.kSRightBack, data.kVRightBack);

    leftNTVoltsController = new FixedVoltsSpeedController("L", leftMotor, leftEncoder::getRate, 4.0);
    rightNTVoltsController = new FixedVoltsSpeedController("R", rightMotor, rightEncoder::getRate, 4.0);

    // Set up the differential drive controllers for the various test modes.
    diffDriveFF = new DifferentialDrive(leftFFController, rightFFController);
    diffDriveNTVolts = new DifferentialDrive(leftNTVoltsController, rightNTVoltsController);

    diffDriveFF.setMaxOutput(MAX_SPEED);
    diffDriveNTVolts.setMaxOutput(MAX_SPEED); // Ignored by FixedVoltsSpeedController

    diffDriveFF.setDeadband(0.12);
    diffDriveNTVolts.setDeadband(0.12);

    setDiffDriveMode(DiffDriveMode.FF);

    setupNetworkTablesListeners();
  }

  public void setDiffDriveMode(DiffDriveMode mode) {
    final String driveModeKey = "Romi-O/DriveMode";

    System.out.println("New drive mode: " + mode.toString());
    System.out.println(data);
    switch (mode) {
      case FF:
        activeDiffDrive = diffDriveFF;
        leftFFController.setParameters (data.kSLeftFwd,   data.kVLeftFwd,   data.kSLeftBack, data.kVLeftBack);
        rightFFController.setParameters(data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
        SmartDashboard.putString(driveModeKey, "FF");
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
          case "drive-left-back-kS":
            data.kSLeftBack = value.getDouble();
            break;
          case "drive-left-back-kV":
            data.kVLeftBack = value.getDouble();
            break;
          case "drive-right-fwd-kS":
            data.kSRightFwd = value.getDouble();
            break;
          case "drive-right-fwd-kV":
            data.kVRightFwd = value.getDouble();
            break;
          case "drive-right-back-kS":
            data.kSRightBack = value.getDouble();
            break;
          case "drive-right-back-kV":
            data.kVRightBack = value.getDouble();
            break;
        }
      }
    }, EntryListenerFlags.kUpdate);
  }

  public void publishParams() {
    String prefix = "Romi-O/";
    SmartDashboard.putNumber(prefix + "drive-left-fwd-kS", data.kSLeftFwd);
    SmartDashboard.putNumber(prefix + "drive-left-fwd-kV", data.kVLeftFwd);

    SmartDashboard.putNumber(prefix + "drive-left-back-kS", data.kSLeftBack);
    SmartDashboard.putNumber(prefix + "drive-left-back-kV", data.kVLeftBack);

    SmartDashboard.putNumber(prefix + "drive-right-fwd-kS", data.kSRightFwd);
    SmartDashboard.putNumber(prefix + "drive-right-fwd-kV", data.kVRightFwd);

    SmartDashboard.putNumber(prefix + "drive-right-back-kS", data.kSRightBack);
    SmartDashboard.putNumber(prefix + "drive-right-back-kV", data.kVRightBack);
  }

  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("ArcadeSpeed", speed);
    SmartDashboard.putNumber("ArcadeRotation", rotation);

    activeDiffDrive.arcadeDrive(speed, rotation, false);
  }

  public void voltDriveLeft(double voltage) {
    SmartDashboard.putNumber("VoltDrive-L", voltage);
    leftMotor.setVoltage(voltage);
  }

  public void voltDriveRight(double voltage) {
    SmartDashboard.putNumber("VoltDrive-R", voltage);
    rightMotor.setVoltage(-voltage);
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
    return ticksToInches(leftEncoder.get());
  }

  public double getRightDistanceInches() {
    return ticksToInches(rightEncoder.get());
  }

  public double getMinDistanceInches() {
    return ticksToInches(Math.min(leftEncoder.get(), rightEncoder.get()));
  }

  public double getMaxDistanceInches() {
    return ticksToInches(Math.max(leftEncoder.get(), rightEncoder.get()));
  }

  public double getAvgDistanceInches() {
    return ticksToInches((leftEncoder.get() + rightEncoder.get()) / 2);
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
    // Only the "active" differential drive feeds its speed controllers.
    // Explicitly feed all of them so nobody starves.
    diffDriveFF.feed();
    diffDriveNTVolts.feed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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

  // Convert wheel encoder ticks to inches based on wheel physical dimensions.
  private double ticksToInches(double ticks) {
    return Math.PI * WHEEL_DIAMETER_INCHES * (ticks / COUNTS_PER_REVOLUTION);
  }
}
