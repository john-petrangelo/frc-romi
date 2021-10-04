/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RomiMap class contains a description of the physical characteristics of
 * the Romi robot.
 */
public final class RomiMap {
    /** The distance between the left and right wheels. */
    public final static double trackWidthInches = 5.55;

    /** The width of each wheel */
    public final static double wheelWidthInches = 0.25;

    /**
     * The effective track width of the wheels used for turning.
     * 
     * This value was determined empiracally by observing the angular
     * turn rate achieved for a known wheel speed. This number differs
     * from the actual track width due to real-world complications like
     * friction, wheel thickness, etc.
     */
    public final static double turnTrackWidthInches = 6.01;

    /**
     * The number of ticks per revolution for the Romi wheel sensors.
     */
    public static final double COUNTS_PER_REVOLUTION = 1440.0;

    /**
     * The diameter of the Romi wheels.
     */
	public static final double WHEEL_DIAMETER_INCHES = 2.75;

    /**
     * The number of inches travelled per tick of the Romi wheel sensors.
     */
	public static final double INCHES_PER_TICK = Math.PI * WHEEL_DIAMETER_INCHES / COUNTS_PER_REVOLUTION;

    /**
     * The maximum speed of the Romi wheels in inches per second.
     */
    public static final double MAX_SPEED = 20.0;

    /**
     * The maximum turn rate of the Romi in degrees per second
     * <br><br>
     * Note: this was collected emperically, based on setting max speed to 20 in/sec.
     */
    public static final double MAX_TURN_RATE = 410.0;
    
    /**
     * The PWM ID for the Romi left wheel motor.
     */
    public static final int LEFT_WHEEL_MOTOR_ID = 0;

    /**
     * The PWM ID for the Romi right wheel motor.
     */
    public static final int RIGHT_WHEEL_MOTOR_ID = 1;

    /**
     * The deadband to apply to joystick control inputs. Changes within the deadband are ignored.
     */
    public static final double CONTROLS_DEADBAND = 0.12;
}
