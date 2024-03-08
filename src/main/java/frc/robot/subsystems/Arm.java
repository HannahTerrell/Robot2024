package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax m_armMotor = new CANSparkMax(16, MotorType.kBrushless);
    private RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private PIDController m_positionController = new PIDController(0.07, 0, 0);
    private boolean m_stopped;

    // be careful setting this to low (slow) because it also prevents slowing down.
    private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(6);

    private final double MAX_SETPOINT = 50;

    public Arm() {
      super();
      m_armMotor.setIdleMode(IdleMode.kBrake);
      m_armMotor.getEncoder().setPosition(0);
      setPositionDown();
    }

    public void setPositionDown() {
        m_positionController.setSetpoint(0);
        m_stopped = false;
    }

    public void setAimpointAmp() {
        // TODO: what posotion should this be?
        m_positionController.setSetpoint(MAX_SETPOINT);
        m_stopped = false;
    }

    public void adjustAim(double rate) {
        m_positionController.setSetpoint(
            MathUtil.clamp(m_positionController.getSetpoint() + rate, 0, MAX_SETPOINT)
        );
    }

    public void setAimpointSpeaker(double distance) {
        // 2m - 12.75, 13.3

        // this is a linear formula, and doesn't account for a ballistic arc.
        // it would be good to have a couple distances with encoder measurements here.
        var setpoint = (distance / 6) * 20;
        setpoint = MathUtil.clamp(setpoint, 0, MAX_SETPOINT);

        m_positionController.setSetpoint(setpoint);
        m_stopped = false;
    }

    public boolean getIsAtSetpoint() {
        // TODO: does this work right?
        return Math.abs(m_positionController.getPositionError()) < .01;
    }

    public void stop() {
        m_stopped = true;
        m_armMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (!m_stopped)
        {
            var speed = m_positionController.calculate(m_armEncoder.getPosition());
            speed = MathUtil.clamp(speed, -1, 1);
            speed = m_rateLimiter.calculate(speed);

            m_armMotor.set(speed);
        }

        SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint", m_positionController.getSetpoint());
        SmartDashboard.putNumber("Arm Speed", m_armMotor.get());
    }
}
