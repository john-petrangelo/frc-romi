/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveForward extends CommandBase {
    private RomiDrivetrain drivetrain;
    private double distance;

    public DriveForward(RomiDrivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
        setName("DriveForward " + distance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.arcadeDrive(0.5, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.getRightDistanceInch() > distance && drivetrain.getLeftDistanceInch() > distance;
    }
}
