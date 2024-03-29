package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WriteMessageCmd extends CommandBase {
    private final String message;

    /**
     * Create a command that writes the specified message to the console.
     */
    public WriteMessageCmd(String message) {
        this.message = message;
    }

    @Override
    public boolean isFinished() {
        System.out.printf("%s\n", this.message);
        return true;
    }
}
