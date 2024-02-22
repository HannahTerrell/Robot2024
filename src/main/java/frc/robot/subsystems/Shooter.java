package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX m_shootMotor = new TalonFX(14);
  private TalonFX m_shootFollower = new TalonFX(15);

  public Shooter() {
    super();
    m_shootMotor.setNeutralMode(NeutralModeValue.Brake);
    m_shootFollower.setControl(new Follower(m_shootMotor.getDeviceID(), false));
  }

  public void shootSpeaker() {
    m_shootMotor.set(0.5);
  }

  public void shootAmp() {
    m_shootMotor.set(0.1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", m_shootMotor.get());
  }

}
