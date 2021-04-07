package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CalibrationLogger extends InstantCommand {
    /**
     * Creates a command that reads previously recorded calibration results from
     * the SmartDashboard and writes them to the log.
     */
    public CalibrationLogger() {
        super(CalibrationLogger::logCalibrationData);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    private static void logCalibrationData() {
        double lf = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(LEFT_FWD)", -1.0);
        double lb = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(LEFT_BACK)", -1.0);
        double rf = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(RIGHT_FWD)", -1.0);
        double rb = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(RIGHT_BACK)", -1.0);

        System.out.printf("---------------------------------------------\n");
        System.out.printf("MINIMUM VOLTAGE REQUIRED TO BEGIN MOVING\n");
        System.out.printf("Left Forward:  %5.3f    Right Forward:  %5.3f\n", lf, rf);
        System.out.printf("Left Backward: %5.3f    Right Backward: %5.3f\n", lb, rb);
        System.out.printf("---------------------------------------------\n");
    }
}
