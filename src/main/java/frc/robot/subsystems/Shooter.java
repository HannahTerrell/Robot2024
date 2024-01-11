package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shootMotor = new CANSparkMax(11, MotorType.kBrushless);
  private double m_speed;

    public Shooter() {
      super();
      m_shootMotor.setIdleMode(IdleMode.kBrake);
    }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  @Override
  public void periodic() {
    m_shootMotor.set(m_speed);
    SmartDashboard.putNumber("Shooter Speed", m_shootMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
