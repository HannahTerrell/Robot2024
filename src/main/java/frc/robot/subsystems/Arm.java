package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax m_armMotor = new CANSparkMax(16, MotorType.kBrushless);
    private double m_speed;

    public Arm() {
      super();
      m_armMotor.setIdleMode(IdleMode.kBrake);
      m_armMotor.getEncoder().setPosition(0);
    }

    public void setSpeed(double speed) {
        m_speed = speed;
    }

    public void moveToTag(Limelight limelight) {
        double limelight_ty = limelight.getTY().getDouble(0);
        if (Math.abs(limelight_ty) > 2) {
            m_armMotor.set(limelight_ty * 0.01);
        }
    }

    public void stop() {
        m_armMotor.set(0);
    }

    @Override
    public void periodic() {
        m_armMotor.set(m_speed);
        SmartDashboard.putNumber("Arm Position: ", m_armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Speed: ", m_speed);
    }

}
