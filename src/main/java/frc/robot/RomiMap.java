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
    public final static double trackWidthInches = 5.5;

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
}
