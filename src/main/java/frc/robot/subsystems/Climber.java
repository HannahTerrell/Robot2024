package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax m_leftClimbMotor = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax m_rightClimbMotor = new CANSparkMax(13, MotorType.kBrushless);
  private Timer m_timer = new Timer();

    public Climber() {
      super();
      m_leftClimbMotor.setIdleMode(IdleMode.kBrake);
      m_rightClimbMotor.setIdleMode(IdleMode.kBrake);
    }

    public void climbUp() {
      m_timer.start();
      m_leftClimbMotor.set(0.4);
      m_rightClimbMotor.set(0.4);
      if (m_timer.hasElapsed(2)) {
        m_leftClimbMotor.set(0);
        m_rightClimbMotor.set(0);
      }
    }

    public void climbDown() {
      m_timer.start();
      m_leftClimbMotor.set(-0.4);
      m_rightClimbMotor.set(-0.4);
      if (m_timer.hasElapsed(2)) {
        m_leftClimbMotor.set(0);
        m_rightClimbMotor.set(0);
      }
    }

  @Override
  public void periodic() {
    
  }


}
