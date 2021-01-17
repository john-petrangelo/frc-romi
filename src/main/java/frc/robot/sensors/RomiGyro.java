// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class RomiGyro {
  private SimDouble m_simRateX;
  private SimDouble m_simRateY;
  private SimDouble m_simRateZ;
  private SimDouble m_simAngleX;
  private SimDouble m_simAngleY;
  private SimDouble m_simAngleZ;

  private double m_angleXOffset;
  private double m_angleYOffset;
  private double m_angleZOffset;

  /** Create a new RomiGyro. */
  public RomiGyro() {
    SimDevice gyroSimDevice = SimDevice.create("Gyro:RomiGyro");
    if (gyroSimDevice != null) {
      gyroSimDevice.createBoolean("init", Direction.kOutput, true);
      m_simRateX = gyroSimDevice.createDouble("rate_x", Direction.kInput, 0.0);
      m_simRateY = gyroSimDevice.createDouble("rate_y", Direction.kInput, 0.0);
      m_simRateZ = gyroSimDevice.createDouble("rate_z", Direction.kInput, 0.0);

      m_simAngleX = gyroSimDevice.createDouble("angle_x", Direction.kInput, 0.0);
      m_simAngleY = gyroSimDevice.createDouble("angle_y", Direction.kInput, 0.0);
      m_simAngleZ = gyroSimDevice.createDouble("angle_z", Direction.kInput, 0.0);
    }
  }

  /**
   * Get the rate of turn in radians-per-second around the X-axis.
   *
   * @return rate of turn in radians-per-second
   */
  public double getRateRoll() {
    if (m_simRateX != null) {
      return -Math.toRadians(m_simRateX.get());
    }

    return 0.0;
  }

  /**
   * Get the rate of turn in radians-per-second around the Y-axis.
   *
   * @return rate of turn in radians-per-second
   */
  public double getRatePitch() {
    if (m_simRateY != null) {
      return -Math.toRadians(m_simRateY.get());
    }

    return 0.0;
  }

  /**
   * Get the rate of turn in radians-per-second around the Z-axis.
   *
   * @return rate of turn in radians-per-second
   */
  public double getRateYaw() {
    if (m_simRateZ != null) {
      return -Math.toRadians(m_simRateZ.get());
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around X-axis in radians
   */
  public double getAngleRoll() {
    if (m_simAngleX != null) {
      return -Math.toRadians(m_simAngleX.get() - m_angleXOffset);
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around Y-axis in radians
   */
  public double getAnglePitch() {
    if (m_simAngleY != null) {
      return -Math.toRadians(m_simAngleY.get() - m_angleYOffset);
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the Z-axis.
   *
   * @return current angle around Z-axis in radians
   */
  public double getAngleYaw() {
    if (m_simAngleZ != null) {
      return -Math.toRadians(m_simAngleZ.get() - m_angleZOffset);
    }

    return 0.0;
  }

  /** Reset the gyro angles to 0. */
  public void reset() {
    if (m_simAngleX != null) {
      m_angleXOffset = m_simAngleX.get();
      m_angleYOffset = m_simAngleY.get();
      m_angleZOffset = m_simAngleZ.get();
    }
  }
}
