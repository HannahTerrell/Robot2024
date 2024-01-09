package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private double m_speed;
    private CANSparkMax m_climbMotor = new CANSparkMax(10, MotorType.kBrushless);

    public Climber() {
      super();
      m_climbMotor.setIdleMode(IdleMode.kBrake);
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
