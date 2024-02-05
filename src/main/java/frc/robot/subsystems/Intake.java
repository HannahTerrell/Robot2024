package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax m_intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax m_feedMotor = new CANSparkMax(10, MotorType.kBrushless);
  private DigitalInput m_sensor = new DigitalInput(0);
  private double m_speed;

  public Intake() {
    super();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_feedMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  public void feed() {
    while (m_sensor.get()) {
      m_feedMotor.set(0.4);
    }
  }

  @Override
  public void periodic() {
      m_intakeMotor.set(m_speed);
      SmartDashboard.putNumber("Intake Speed", m_intakeMotor.get());
  }
}
