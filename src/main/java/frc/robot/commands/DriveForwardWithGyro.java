package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveForwardWithGyro extends PIDCommand {
    private final RomiDrivetrain drivetrain;
    private final RomiGyro gyro;
    private final double inches;

    private static final double SPEED = 1.0;
    private static double kP = 0.08;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public DriveForwardWithGyro(double inches, RomiDrivetrain drivetrain, RomiGyro gyro) {
        super(
            new PIDController(kP, kI, kD),
            () -> -gyro.getAngleYaw(),
            0,
            output -> {
                System.out.printf("output=%3.1f\n", output);
                drivetrain.arcadeDrive(Math.signum(inches) * SPEED, output);
            },
            drivetrain, gyro);
            
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.inches = inches;

        addRequirements(drivetrain);
        SendableRegistry.setName(getController(), "HeadingPID");
        setName("DriveForwardWithGyro(" + inches + ")");
        SmartDashboard.putNumber("kP", kP);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        kP = SmartDashboard.getNumber("kP", 0.0);
        getController().setP(kP);
        System.out.printf("%s initialize kP=%f\n", getName(), getController().getP());
        drivetrain.resetEncoders();
        gyro.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished =
            Math.abs(drivetrain.getRightDistanceInches()) > Math.abs(inches) ||
            Math.abs(drivetrain.getLeftDistanceInches())  > Math.abs(inches);

        System.out.printf("%s is %sfinished, left=%3.1f right=%3.1f heading=%3.1f\n",
            getName(), isFinished ? "" : "not ",
            drivetrain.getLeftDistanceInches(), drivetrain.getRightDistanceInches(),
            -gyro.getAngleYaw());

        return isFinished;
    }
}
