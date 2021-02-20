/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.speedcontrollers.FeedforwardSpeedController;
import frc.robot.speedcontrollers.NTVoltsSpeedController;
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

  // Keep a filter for each side's driving speed.
  private LinearFilter leftDriveSpeed = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter rightDriveSpeed = LinearFilter.singlePoleIIR(0.1, 0.02);

  private final FeedforwardSpeedController leftFFController;
  private final FeedforwardSpeedController rightFFController;

  private final PIDSpeedController leftPIDController;
  private final PIDSpeedController rightPIDController;

  private final PIDFSpeedController leftPIDFController;
  private final PIDFSpeedController rightPIDFController;

  private final NTVoltsSpeedController leftNTVoltsController;
  private final NTVoltsSpeedController rightNTVoltsController;

  // Set up the differential drive controllers.
  private final DifferentialDrive diffDriveRaw;
  private final DifferentialDrive diffDriveFF;
  private final DifferentialDrive diffDrivePID;
  private final DifferentialDrive diffDrivePIDF;
  private final DifferentialDrive diffDriveNTVolts;
  private DifferentialDrive activeDiffDrive;

  public enum DiffDriveMode {
    RAW, FF, PID, PIDF, NT_VOLTS
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

    void dump() {
      System.out.printf("Fwd  Left  (kS, kV, kP) = (%5.3f, %5.3f, %5.3f)\n", kSLeftFwd, kVLeftFwd, kPLeftFwd);
      System.out.printf("Fwd  Right (kS, kV, kP) = (%5.3f, %5.3f, %5.3f)\n", kSRightFwd, kVRightFwd, kPRightFwd);
      System.out.printf("Back Left  (kS, kV, kP) = (%5.3f, %5.3f, %5.3f)\n", kSLeftBack, kVLeftBack, kPLeftBack);
      System.out.printf("Back Right (kS, kV, kP) = (%5.3f, %5.3f, %5.3f)\n", kSRightBack, kVRightBack, kPRightBack);
    }
  };
  
  private final static Characteristics dataJan17 = new Characteristics(
      1.18, 0.170, 0.000340,
      1.81, 0.134, 0.000224,
      1.41, 0.149, 0.000469,
      1.30, 0.160, 0.0000757);

  private final static Characteristics dataJan29OnBox = new Characteristics(
      1.44, 0.192, 0.00248,
      1.43, 0.182, 0.00383,
      1.27, 0.213, 0.00404,
      1.17, 0.201, 0.00320);

  // dataJan17Groomed(
  //   0.531, 0.241, 10,
  //   0.758, 0.228, 10,
  //   0.426, 0.261, 10,
  //   0.534, 0.254, 10),
  private final static Characteristics dataJan17Groomed = new Characteristics(
      0.531, 0.241, 0.25,
      0.550, 0.227, 0.25,
      0.534, 0.283, 0.25,
      0.426, 0.254, 0.25);
      
  // From https://github.com/bb-frc-workshops/romi-examples
  private final static Characteristics dataFromExample = new Characteristics(
    0.929, 6.33, 0.085,
    0.929, 6.33, 0.085,
    0.929, 6.33, 0.085,
    0.929, 6.33, 0.085);

  private Characteristics data = dataJan17Groomed;

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

    leftNTVoltsController = new NTVoltsSpeedController("L", leftMotor, leftEncoder::getRate, 1.0);
    rightNTVoltsController = new NTVoltsSpeedController("R", rightMotor, rightEncoder::getRate, 1.0);

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
    System.out.println("setDiffDriveMode " + mode.toString());
    // System.out.println("Characterization Data:");
    // data.dump();

    switch (mode) {
      case RAW:
        activeDiffDrive = diffDriveRaw;
        SmartDashboard.putString("romi-o/controller-mode", "raw");
        break;
      case FF:
        activeDiffDrive = diffDriveFF;
        leftFFController.setParameters (data.kSLeftFwd,   data.kVLeftFwd,   data.kSLeftBack, data.kVLeftBack);
        rightFFController.setParameters(data.kSRightBack, data.kVRightBack, data.kSRightFwd, data.kVRightFwd);
          // data.kSRightFwd, data.kVRightFwd, data.kSRightBack, data.kVRightBack);
        SmartDashboard.putString("romi-o/controller-mode", "FF");
        break;
      case PID:
        activeDiffDrive = diffDrivePIDF;
        leftPIDFController.setParameters( data.kSLeftFwd,   data.kVLeftFwd,   0.0,
                                          data.kSLeftBack,  data.kVLeftBack,  0.0);
        rightPIDFController.setParameters(data.kSRightBack, data.kVRightBack,0.0,
                                          data.kSRightFwd,  data.kVRightFwd, 0.0);
        SmartDashboard.putString("romi-o/controller-mode", "FF + PID(kP=0)");
        break;
        case PIDF:
        activeDiffDrive = diffDrivePIDF;
        leftPIDFController.setParameters( data.kSLeftFwd,   data.kVLeftFwd,   data.kPLeftFwd,
                                          data.kSLeftBack,  data.kVLeftBack,  data.kPLeftBack);
        rightPIDFController.setParameters(data.kSRightBack, data.kVRightBack,data.kPRightBack,
                                          data.kSRightFwd,  data.kVRightFwd, data.kPRightFwd);
        SmartDashboard.putString("romi-o/controller-mode", "FF + PID");
        break;
      case NT_VOLTS:
        activeDiffDrive = diffDriveNTVolts;
        double volts = SmartDashboard.getNumber("romi-o/set-volts", 0.0);
        leftNTVoltsController.setParameters(volts);
        rightNTVoltsController.setParameters(volts);
        SmartDashboard.putString("romi-o/controller-mode", "NT Volts");
        break;
    }
    
    resetEncoders();
  }

  private void setupNetworkTablesListeners() {
    NetworkTableInstance nti = NetworkTableInstance.getDefault();
    NetworkTable ntTable = nti.getTable("SmartDashboard/romi-o");

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
    String prefix = "romi-o/";
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
    SmartDashboard.putNumber("arcade speed", speed);
    SmartDashboard.putNumber("arcade rotation", rotation);
    activeDiffDrive.arcadeDrive(speed, rotation, false);

    // Only the "active" differential drive feeds its speed controllers.
    // Explicitly feed all of them so nobody starves.
    diffDriveRaw.feed();
    diffDriveFF.feed();
    diffDrivePID.feed();
    diffDrivePIDF.feed();

    
    // Update the drive speed filters and report to the SmartDashboard.
    SmartDashboard.putNumber("left-speed", leftDriveSpeed.calculate(getLeftRate()));
    SmartDashboard.putNumber("right-speed", rightDriveSpeed.calculate(getRightRate()));
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return ticksToInches(getLeftEncoderCount());
  }

  public double getRightDistanceInch() {
    return ticksToInches(getRightEncoderCount());
  }

  public double getLeftRateInches() {
    return ticksToInches(leftEncoder.getRate());
  }

  public double getRightRateInches() {
    return ticksToInches(rightEncoder.getRate());
  }

  public double getLeftRate() {
    return leftEncoder.getRate();
  }

  public double getRightRate() {
    return rightEncoder.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
