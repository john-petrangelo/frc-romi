package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that reads previously recorded calibration results from
 * the SmartDashboard and writes them to the log.
 */
public class CalibrationLogger extends CommandBase {
    private final double SLOW_VOLTS;
    private final double FAST_VOLTS;
    private final String SLOW_VOLTS_S;
    private final String FAST_VOLTS_S;


    double lf_start_volts;
    double lb_start_volts;
    double rf_start_volts;
    double rb_start_volts;

    double lf_rate_slow;
    double lb_rate_slow;
    double rf_rate_slow;
    double rb_rate_slow;
    
    double lf_rate_fast;
    double lb_rate_fast;
    double rf_rate_fast;
    double rb_rate_fast;

    public CalibrationLogger(double slowVolts, double fastVolts) {
        SLOW_VOLTS = slowVolts;
        FAST_VOLTS = fastVolts;

        SLOW_VOLTS_S = String.format("%3.1f", SLOW_VOLTS);
        FAST_VOLTS_S = String.format("%3.1f", FAST_VOLTS);
    }

    @Override
    public void initialize() {
        super.initialize();
        logAll();
    }

    private void logAll() {
        gatherData();
        logMinVoltsToBeginMoving();
        logRatesForVoltages();
        logFeedForwardConstants();
    }

    private void gatherData() {
        lf_start_volts = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(LEFT_FWD)", -1.0);
        lb_start_volts = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(LEFT_BACK)", -1.0);
        rf_start_volts = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(RIGHT_FWD)", -1.0);
        rb_start_volts = SmartDashboard.getNumber("Calibration/MinVoltsToStartMoving(RIGHT_BACK)", -1.0);

        lf_rate_slow = SmartDashboard.getNumber("Calibration/LeftRateForVolts("  + SLOW_VOLTS_S +")", -1.0);
        lb_rate_slow = SmartDashboard.getNumber("Calibration/LeftRateForVolts(-" + SLOW_VOLTS_S +")", -1.0);
        rf_rate_slow = SmartDashboard.getNumber("Calibration/RightRateForVolts(" + SLOW_VOLTS_S +")", -1.0);
        rb_rate_slow = SmartDashboard.getNumber("Calibration/RightRateForVolts(-" + SLOW_VOLTS_S +")", -1.0);
        
        lf_rate_fast = SmartDashboard.getNumber("Calibration/LeftRateForVolts(" + FAST_VOLTS_S +")", -1.0);
        lb_rate_fast = SmartDashboard.getNumber("Calibration/LeftRateForVolts(-" + FAST_VOLTS_S +")", -1.0);
        rf_rate_fast = SmartDashboard.getNumber("Calibration/RightRateForVolts(" + FAST_VOLTS_S +")", -1.0);
        rb_rate_fast = SmartDashboard.getNumber("Calibration/RightRateForVolts(-" + FAST_VOLTS_S +")", -1.0);
    }

    private void logMinVoltsToBeginMoving() {
        System.out.printf("---------------------------------------------\n");
        System.out.printf("MINIMUM VOLTAGE REQUIRED TO BEGIN MOVING\n");
        System.out.printf("Left Forward:  %5.3f    Right Forward:  %5.3f\n", lf_start_volts, rf_start_volts);
        System.out.printf("Left Backward: %5.3f    Right Backward: %5.3f\n", lb_start_volts, rb_start_volts);
        System.out.printf("---------------------------------------------\n");
    }

    private void logRatesForVoltages() {
        System.out.printf("-------------------------------------------------\n");
        System.out.printf("RATE FOR VOLTAGE - SLOW\n");
        System.out.printf("Left Forward:   %5.3f    Right Forward:   %5.3f\n", lf_rate_slow, rf_rate_slow);
        System.out.printf("Left Backward: %5.3f    Right Backward: %5.3f\n", lb_rate_slow, rb_rate_slow);
        System.out.printf("-------------------------------------------------\n");
        System.out.printf("RATE FOR VOLTAGE - FAST\n");
        System.out.printf("Left Forward:   %5.3f    Right Forward:   %5.3f\n", lf_rate_fast, rf_rate_fast);
        System.out.printf("Left Backward: %5.3f    Right Backward: %5.3f\n", lb_rate_fast, rb_rate_fast);
        System.out.printf("-------------------------------------------------\n");
    }

    private void logFeedForwardConstants() {
        // kV = (volts2 - volts1) / (rate2 - rate1)
        // kS = volts_n - kV * rate_n
        final double lf_kV = ( FAST_VOLTS -  SLOW_VOLTS) / (lf_rate_fast - lf_rate_slow);
        final double lb_kV = (-FAST_VOLTS - -SLOW_VOLTS) / (lb_rate_fast - lb_rate_slow);
        final double rf_kV = ( FAST_VOLTS -  SLOW_VOLTS) / (rf_rate_fast - rf_rate_slow);
        final double rb_kV = (-FAST_VOLTS - -SLOW_VOLTS) / (rb_rate_fast - rb_rate_slow);

        final double lf_kS =    SLOW_VOLTS - lf_kV * lf_rate_slow;
        final double lb_kS = -(-SLOW_VOLTS - lf_kV * lb_rate_slow);
        final double rf_kS =    SLOW_VOLTS - lf_kV * rf_rate_slow;
        final double rb_kS = -(-SLOW_VOLTS - lf_kV * rb_rate_slow);

        System.out.printf("-----------------------------------------------------------\n");
        System.out.printf("FEED FORWARD kS, kV\n");       
        System.out.printf("Left Forward:  %5.3f, %5.3f    Right Forward:  %5.3f, %5.3f\n",
            lf_kS, lf_kV, rf_kS, rf_kV);
        System.out.printf("Left Backward: %5.3f, %5.3f    Right Backward: %5.3f, %5.3f\n",
            lb_kS, lb_kV, rb_kS, rb_kV);
        System.out.printf("-----------------------------------------------------------\n");

        System.out.printf(
            "private final static Characteristics myData = new Characteristics(\n" +
            "    %5.3f, %5.3f,  // LF\n" +
            "    %5.3f, %5.3f,  // LB\n" +
            "    %5.3f, %5.3f,  // RF\n" +
            "    %5.3f, %5.3f); // RB\n",
            lf_kS, lf_kV, lb_kS, lb_kV, rf_kS, rf_kV, rb_kS, rb_kV);
    }

    @Override
    public final boolean isFinished() {
      return true;
    }
}
