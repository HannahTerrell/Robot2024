package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class SysIdCommandPair {
    private Command forward;
    private Command reverse;

    public SysIdCommandPair(Command forward, Command reverse) {
        super();
        this.forward = forward;
        this.reverse = reverse;
    }

    public Command getForward() {
        return forward;
    }

    public Command getReverse() {
        return reverse;
    }
}
