/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.speedcontrollers.FeedforwardSpeedController;
import frc.robot.speedcontrollers.PIDFSpeedController;
import frc.robot.speedcontrollers.PIDSpeedController;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

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

    // Create the speed controllers used for the various test modes.
    leftFFController = new FeedforwardSpeedController(leftMotor,   1.20, 0.0400, 1.46, 0.0345);
    rightFFController = new FeedforwardSpeedController(rightMotor, 1.37 , 0.0397, 1.38, 0.0364);

    leftPIDController = new PIDSpeedController(leftMotor, leftEncoder::getRate, 0.25, 0.0, 0.0);
    rightPIDController = new PIDSpeedController(rightMotor, rightEncoder::getRate, 0.25, 0.0, 0.0);
  
    leftPIDFController = new PIDFSpeedController(leftMotor, leftEncoder::getRate,
      1.20, 0.0400, 1.46, 0.0345,
      5.23e-6, 0.0, 0.0);
    rightPIDFController = new PIDFSpeedController(rightMotor, rightEncoder::getRate,
      1.37 , 0.0397, 1.38, 0.0364,
      5.04e-6, 0.0, 0.0);

    // Set up the differential drive controllers for the various test modes.
    diffDriveRaw = new DifferentialDrive(leftMotor, rightMotor);
    diffDriveFF = new DifferentialDrive(leftFFController, rightFFController);
    diffDrivePID = new DifferentialDrive(leftPIDController, rightPIDController);
    diffDrivePIDF = new DifferentialDrive(leftPIDFController, rightPIDFController);

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
    return Math.PI * kWheelDiameterInch * (ticks / kCountsPerRevolution);
  }
}
