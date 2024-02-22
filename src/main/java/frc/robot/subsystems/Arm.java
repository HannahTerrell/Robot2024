package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax m_armMotor = new CANSparkMax(16, MotorType.kBrushless);
    private Timer m_armTimer = new Timer();

    public Arm() {
      super();
      m_armMotor.setIdleMode(IdleMode.kBrake);
    }

    public void moveUp() {
        m_armTimer.start();
        m_armMotor.set(0.5);
        if (m_armTimer.hasElapsed(2.0)) {
            m_armMotor.set(0);
        }
        m_armTimer.reset();
    }

    public void moveDown() {
        m_armTimer.start();
        m_armMotor.set(-0.5);
        if (m_armTimer.hasElapsed(2.0)) {
            m_armMotor.set(0);
        }
        m_armTimer.reset();
    }

    public void moveToTag(Limelight limelight) {
        double limelight_ty = limelight.getTY().getDouble(0);
        if (Math.abs(limelight_ty) > 2) {
            m_armMotor.set(limelight_ty * 0.01);
        }
    }

}
