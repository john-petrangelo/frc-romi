package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WriteMessage extends CommandBase {
    private final String message;

    public WriteMessage(String message) {
        this.message = message;
    }

    @Override
    public void initialize() {
        System.out.printf("%s\n", this.message);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
