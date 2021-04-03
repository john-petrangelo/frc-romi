package frc.robot.commands.calibration;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class FindFFkS extends CommandBase {
    public enum Side {
        LEFT_FWD, LEFT_BACK, RIGHT_FWD, RIGHT_BACK
    }

    public class Derivative {
        private double previousValue = 0.0;
        private double currentRate = 0.0;
        private long lastCalculateTime = System.currentTimeMillis();

        public double calculate(double newValue) {
            long now = System.currentTimeMillis();
            double deltaT = (now - lastCalculateTime) / 1000.0;

            // System.out.println(String.format("D new=%5.3f prev=%5.3f dt=%5.3f",
            //     newValue, previousValue, deltaT));
            currentRate = (newValue - previousValue) / deltaT;

            lastCalculateTime = now;
            previousValue = newValue;

            return currentRate;
        }

        public double get() {
            return currentRate;
        }
    }

    private static final long STEP_DURATION_MS = 200;
    private static final double STEP_SIZE_VOLTS = 0.02;
    private static final double STARTING_VOLTAGE = 2.0;

    private RomiDrivetrain drivetrain;
    private long stepStartTime;
    private double currentStepVoltage;

    private Supplier<Double> distance;
    private Supplier<Double> distanceEst;
    private Supplier<Double> speedEst;
    private Supplier<Double> accelEst;
    private Consumer<Double> drive;
    private Derivative currentSpeed = new Derivative();

    /**
     * Creates a command that finds the minimum voltage needed to move the robot.
     * <p>
     * Starts with very low voltage and slow ramp up until
     * motion is detected. Once motion is detected, stop and report the voltage.
     */
    public FindFFkS(Side side, RomiDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        switch(side) {
            case LEFT_FWD:
            distance = () -> drivetrain.getLeftDistanceInches();
            drive = (volts) -> drivetrain.voltDriveLeft(volts);
            break;
        case LEFT_BACK:
            distance = () -> -drivetrain.getLeftDistanceInches();
            drive = (volts) -> drivetrain.voltDriveLeft(-volts);
            break;
        case RIGHT_FWD:
            distance = () -> drivetrain.getRightDistanceInches();
            drive = (volts) -> drivetrain.voltDriveRight(volts);
            break;
        case RIGHT_BACK:
            distance = () -> -drivetrain.getRightDistanceInches();
            drive = (volts) -> drivetrain.voltDriveRight(-volts);
            break;
        }

        setName(String.format("%s(%s)", getName(), side.toString()));
    }

    @Override
    public void initialize() {
        super.initialize();
        stepStartTime = System.currentTimeMillis();
        currentStepVoltage = STARTING_VOLTAGE;

        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        // Update the speed measurement.
        currentSpeed.calculate(distance.get());

        // If it's too soon, don't bother checking.
        long now = System.currentTimeMillis();

        // Have we tried this voltage long enough?
        if (now >= stepStartTime + STEP_DURATION_MS) {
            stepStartTime += STEP_DURATION_MS;
            currentStepVoltage -= STEP_SIZE_VOLTS;

            System.out.printf("%3d %s Stepping voltage V=%5.3f dist=%5.3f speed=%5.3f\n",
                now % 1000, getName(), currentStepVoltage, distance.get(), currentSpeed.get());
        }

        // Time,distance,speed,volts
        // System.out.println(String.format("DATA,%d,%5.3f,%5.3f,%5.3f",
        //     now, distance.get(), currentSpeed.get(), distanceEst.get()));

        drive.accept(currentStepVoltage);
        return;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%s end voltage=%5.3f distance=%5.3f\n",
            getName(), currentStepVoltage, distance.get());
        SmartDashboard.putNumber("Calibration/" + getName(), currentStepVoltage);

        drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return currentStepVoltage < 0.0;
    }
}
