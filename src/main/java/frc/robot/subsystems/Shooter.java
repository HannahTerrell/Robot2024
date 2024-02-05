package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shootMotor = new CANSparkMax(13, MotorType.kBrushless);

  public Shooter() {
    super();
    m_shootMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shootSpeaker() {
    m_shootMotor.set(0.8);
  }

  public void shootAmp() {
    m_shootMotor.set(0.3);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", m_shootMotor.get());
  }

}
