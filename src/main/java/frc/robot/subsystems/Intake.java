package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax m_intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax m_intakeFollower = new CANSparkMax(10, MotorType.kBrushless);
  private double m_speed;

  public Intake() {
    super();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_intakeFollower.setIdleMode(IdleMode.kBrake);
    m_intakeFollower.follow(m_intakeMotor, true);
  }

  public void intakeAndFeed(double speed) {
    m_speed = speed;
  }

  @Override
  public void periodic() {
      m_intakeMotor.set(m_speed);
      SmartDashboard.putNumber("Intake Speed", m_intakeMotor.get());
  }
}
