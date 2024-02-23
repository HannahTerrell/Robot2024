package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax m_intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax m_intakeFollower = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax m_feedMotor = new CANSparkMax(11, MotorType.kBrushless);
  private DigitalInput m_sensor = new DigitalInput(0);
  private double m_intakeSpeed;
  private double m_feedSpeed;

  public Intake() {
    super();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
   //m_intakeFollower.follow(m_intakeMotor);
    m_feedMotor.setIdleMode(IdleMode.kBrake);
  }

  public void intakeAndFeed(double speed) {
    m_intakeSpeed = speed;
    if (m_sensor.get()) {
      m_feedSpeed = speed;
    }

  }

  @Override
  public void periodic() {
      m_intakeMotor.set(m_intakeSpeed);
      m_intakeFollower.set(m_intakeSpeed);
      m_feedMotor.set(m_feedSpeed);
      SmartDashboard.putNumber("Intake Speed", m_intakeMotor.get());
  }
}
