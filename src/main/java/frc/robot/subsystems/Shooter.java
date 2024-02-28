package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX m_shootMotor = new TalonFX(14);
  private TalonFX m_shootFollower = new TalonFX(15);
  private CANSparkMax m_feedMotor = new CANSparkMax(11, MotorType.kBrushless);

  public Shooter() {
    super();
    m_shootFollower.setControl(new Follower(m_shootMotor.getDeviceID(), true));
    m_shootMotor.setNeutralMode(NeutralModeValue.Brake);
    m_shootFollower.setNeutralMode(NeutralModeValue.Brake);
    m_feedMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shootSpeaker() {
    m_feedMotor.set(0.5);
    m_shootMotor.set(-0.7);
  }

  public void shootAmp() {
    m_feedMotor.set(0.5);
    m_shootMotor.set(-0.3);
  }

  public void feedOnly() {
    m_feedMotor.set(0.5);
  }

  public void shootSpeakerOnly() {
    m_shootMotor.set(-0.7);
  }

  public void shootAmpOnly() {
    m_shootMotor.set(-0.3);
  }

  public void stop() {
    m_shootMotor.set(0);
    m_feedMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", m_shootMotor.get());
  }

}
