/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.speedcontrollers.FeedforwardSpeedController;
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

  // Set up the differential drive controllers.
  private final DifferentialDrive diffDriveRaw;
  private final DifferentialDrive diffDriveFF;
  private final DifferentialDrive diffDrivePID;
  private final DifferentialDrive diffDrivePIDF;
  private DifferentialDrive activeDiffDrive;

  public enum DiffDriveMode {
    RAW, FF, PID, PIDF
  }

  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain() {
    leftEncoder.setDistancePerPulse(ticksToInches(1));
    rightEncoder.setDistancePerPulse(ticksToInches(1));

    // Create the speed controllers used for the various test modes.
    // leftFFController = new FeedforwardSpeedController(leftMotor,   1.20, 0.0400, 1.46, 0.0345);
    // rightFFController = new FeedforwardSpeedController(rightMotor, 1.37 , 0.0397, 1.38, 0.0364);
    leftFFController = new FeedforwardSpeedController(leftMotor,   1.18, 0.17, 1.81, 0.134);
    rightFFController = new FeedforwardSpeedController(rightMotor, 1.41, 0.149, 1.3, 0.16);

    leftPIDController = new PIDSpeedController(leftMotor, leftEncoder::getRate, 0.25, 0.0, 0.0);
    rightPIDController = new PIDSpeedController(rightMotor, rightEncoder::getRate, 0.25, 0.0, 0.0);
  
    leftPIDFController = new PIDFSpeedController(leftMotor, leftEncoder::getRate,
      1.18, 0.17, 1.81, 0.134,
      0.0003400, 0.0002240);
    rightPIDFController = new PIDFSpeedController(rightMotor, rightEncoder::getRate,
      1.41, 0.149, 1.3, 0.16,
      0.0004690, 0.0000757);

    // Set up the differential drive controllers for the various test modes.
    diffDriveRaw = new DifferentialDrive(leftMotor, rightMotor);
    diffDriveFF = new DifferentialDrive(leftFFController, rightFFController);
    diffDrivePID = new DifferentialDrive(leftPIDController, rightPIDController);
    diffDrivePIDF = new DifferentialDrive(leftPIDFController, rightPIDFController);

    diffDriveFF.setMaxOutput(MAX_SPEED);
    diffDrivePID.setMaxOutput(MAX_SPEED);
    diffDrivePIDF.setMaxOutput(MAX_SPEED);

    // Default to "raw" mode.
    setDiffDriveMode(DiffDriveMode.RAW);
  }

  public void setDiffDriveMode(DiffDriveMode mode) {
    System.out.println("setDiffDriveMode " + mode.toString());
    switch (mode) {
      case RAW:
        activeDiffDrive = diffDriveRaw;
        SmartDashboard.putString("controller-mode", "raw");
        break;
      case FF:
        activeDiffDrive = diffDriveFF;
        SmartDashboard.putString("controller-mode", "FF");
        break;
      case PID:
        activeDiffDrive = diffDrivePID;
        SmartDashboard.putString("controller-mode", "PID");
        break;
      case PIDF:
        activeDiffDrive = diffDrivePIDF;
        SmartDashboard.putString("controller-mode", "FF + PID");
        break;
    }
    
    resetEncoders();
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
