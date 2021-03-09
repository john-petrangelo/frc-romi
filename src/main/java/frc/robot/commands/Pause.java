package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Pause extends CommandBase {
    private final long durationMS;
    private long startTimeMS = 0;

    /**
     * Create a command to pause for the specified number of seconds.
     */
    public Pause(double seconds) {
        this.durationMS = (long)(seconds * 1000);
        setName(String.format("Pause(%5.3fs)", seconds));
    }

    @Override
    public void initialize() {
        startTimeMS = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        final long now = System.currentTimeMillis();
        final boolean isFinished = now > startTimeMS + durationMS;

        if (isFinished) {
            System.out.printf("%s is %sfinished, elapsed=%5.3fs\n",
                getName(), isFinished ? "" : "not ", (now - startTimeMS) / 1000.0);
        }

        return isFinished;
    }
}
