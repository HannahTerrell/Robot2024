package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmUp extends Command {
    private Arm m_arm;
    private Timer m_armTimer = new Timer();

    public ArmUp(Arm arm) {
        super();
        m_arm = arm;    
    }

    @Override
    public void initialize() {
        m_armTimer.reset();
        m_armTimer.start();
    }

    @Override
    public void execute() {
        m_arm.setSpeed(1);
    }

    @Override
    public boolean isFinished() {
        return m_armTimer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_arm.setSpeed(0);
    }

}
