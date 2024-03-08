package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TargetType;

public class Shooter extends SubsystemBase {
  private TalonFX m_shootMotor = new TalonFX(14);
  private TalonFX m_shootFollower = new TalonFX(15);
  private CANSparkMax m_feedMotor = new CANSparkMax(11, MotorType.kBrushless);

  public Shooter() {
    super();
    m_shootMotor.setNeutralMode(NeutralModeValue.Brake);
    m_shootFollower.setControl(new Follower(m_shootMotor.getDeviceID(), true));
    m_shootFollower.setNeutralMode(NeutralModeValue.Brake);
    m_feedMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shootSpeaker() {
    m_feedMotor.set(1);
    shootSpeakerOnly();
  }

  public void shootAmp() {
    m_feedMotor.set(1);
    m_shootMotor.set(0.3);
  }

  public void feedOnly() {
    m_feedMotor.set(1);
  }

  public void shootSpeakerOnly() {
    m_shootMotor.set(1);
  }

  public void shootAmpOnly() {
    m_shootMotor.set(0.3);
  }

  public void shoot(TargetType target) {
    switch (target) {
      case AMP:
        m_shootMotor.set(-0.3);
        break;

      case SPEAKER:
        m_shootMotor.set(-0.7);
        break;

      default:
        // can't shoot at that until you tell me how
        break;
    }
  }

  public void backfeed() {
    m_feedMotor.set(-0.25);
  }

  public void stop() {
    m_shootMotor.set(0);
    m_feedMotor.set(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter Speed/1", m_shootMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Speed/2", m_shootFollower.getVelocity().getValueAsDouble());
  }

}
