package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDown extends Command {
    private Arm m_arm;
    private Timer m_armTimer = new Timer();

    public ArmDown(Arm arm) {
        super();
        m_arm = arm;    
    }

    @Override
    public void initialize() {
        m_armTimer.start();
    }

    @Override
    public void execute() {
        m_arm.setSpeed(-0.5);
    }

    @Override
    public boolean isFinished() {
        return m_armTimer.hasElapsed(4);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_arm.setSpeed(0);
        m_armTimer.reset();
    }

}
