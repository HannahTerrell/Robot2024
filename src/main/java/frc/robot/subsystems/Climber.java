package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private double m_speed;
  private TalonFX m_climbMotor = new TalonFX(10);

    public Climber() {
      super();
      m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  @Override
  public void periodic() {
    m_climbMotor.set(m_speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
