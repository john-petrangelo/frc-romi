/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class ArcadeDriveCmd extends CommandBase {
    private RomiDrivetrain drivetrain;
    private Supplier<Double> speedSupplier;
    private Supplier<Double> rotateSupplier;

    public ArcadeDriveCmd(RomiDrivetrain drivetrain, Supplier<Double> speedSupplier, Supplier<Double> rotateSupplier) {
        this.drivetrain = drivetrain;
        this.speedSupplier = speedSupplier;
        this.rotateSupplier = rotateSupplier;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.arcadeDrive(speedSupplier.get(), rotateSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
