package frc.robot.sensors;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * A base class for sensors that handles Sendable registration in the constructor.
 */
public abstract class SensorBase implements Sendable {
  public SensorBase() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    SendableRegistry.addLW(this, name);
  }

  /**
   * Gets the name of this Sensor.
   */
  @Override
  public String getName() {
    return SendableRegistry.getName(this);
  }

  /**
   * Sets the name of this Sensor.
   */
  @Override
  public void setName(String name) {
    SendableRegistry.setName(this, name);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Sensor");
  }
}
