package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax m_leftClimbMotor = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax m_rightClimbMotor = new CANSparkMax(13, MotorType.kBrushless);
  private double m_speed;
  private String m_side = "";

    public Climber() {
      super();
      m_leftClimbMotor.setIdleMode(IdleMode.kBrake);
      m_rightClimbMotor.setIdleMode(IdleMode.kBrake);
    }

    public CANSparkMax getDesiredMotor(String side) {
      CANSparkMax motorToSet = m_rightClimbMotor;
      if (side.equals("left")) {
        motorToSet = m_leftClimbMotor;
      } else if (side.equals("right")) {
        motorToSet = m_rightClimbMotor;
      }
      return motorToSet;
    }

    public void climb(String side, double speed) {
      m_side = side;
      m_speed = speed;
    }

    public void stop() {
      m_leftClimbMotor.set(0);
      m_rightClimbMotor.set(0);
    }

    @Override
    public void periodic() {
      getDesiredMotor(m_side).set(m_speed);
    }


}
