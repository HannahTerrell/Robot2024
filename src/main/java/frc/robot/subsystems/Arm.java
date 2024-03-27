package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax m_armMotor = new CANSparkMax(16, MotorType.kBrushless);
    private RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private ArmAimHelper m_armAimHelper = new ArmAimHelper();
    private NetworkTableEntry m_armSetpointAdjustmentEntry;
    private PIDController m_positionController = new PIDController(0.1, 0.03, 0);
    private boolean m_stopped;

    // be careful setting this to low (slow) because it also prevents slowing down.
    private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(6);

    private final double MAX_SETPOINT = 45;
    private final double AIM_ADJUSTMENT = 0.10;


    public Arm() {
        super();
        m_armMotor.setIdleMode(IdleMode.kCoast);
        m_armMotor.getEncoder().setPosition(0);
        setPositionDown();

        SmartDashboard.putData("Arm/Position Controller", m_positionController);
        SmartDashboard.putNumber("Arm/Position Adjustment", 0);

        m_armSetpointAdjustmentEntry = SmartDashboard.getEntry("Arm/Position Adjustment");
        m_armSetpointAdjustmentEntry.setDefaultDouble(0);
        m_armSetpointAdjustmentEntry.setPersistent();
    }

    public void autoAimAdjustDown() {
        // name appears reversed, but to aim lower, we need to bump our setpoint up
        m_armSetpointAdjustmentEntry.setDouble(m_armSetpointAdjustmentEntry.getDouble(0) + AIM_ADJUSTMENT);
    }

    public void autoAimAdjustUp() {
        // name appears reversed, but to aim lower, we need to bump our setpoint up
        m_armSetpointAdjustmentEntry.setDouble(m_armSetpointAdjustmentEntry.getDouble(0) - AIM_ADJUSTMENT);
    }

    public void setPositionDown() {
        m_positionController.setSetpoint(0);
        m_stopped = false;
    }

    public void setAimpointAmp() {
        m_positionController.setSetpoint(MAX_SETPOINT);
        m_stopped = false;
    }

    public void adjustAim(double rate) {
        m_positionController.setSetpoint(
                MathUtil.clamp(m_positionController.getSetpoint() + rate, 0, MAX_SETPOINT));
    }

    public void setAimpointSpeaker(double distance) {
        var setpoint = m_armAimHelper.getArmSetpoint(distance) * (1 + m_armSetpointAdjustmentEntry.getDouble(0));
        setpoint = MathUtil.clamp(setpoint, 0, MAX_SETPOINT);

        m_positionController.setSetpoint(setpoint);
        m_stopped = false;
    }

    public boolean getIsAtSetpoint() {
        return Math.abs(m_positionController.getPositionError()) < 0.4;
    }

    public void stop() {
        m_stopped = true;
        m_armMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (!m_stopped) {
            var position = m_armEncoder.getPosition();
            var speed = m_positionController.calculate(position);
            speed = MathUtil.clamp(speed, -1, 1);
            speed = m_rateLimiter.calculate(speed);

            if ((position < 8 && speed < 0) || (position > 42 && speed > 0)) {
                speed = MathUtil.clamp(speed, -.10, 0.25);
            }

            m_armMotor.set(speed);
        }

        SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint", m_positionController.getSetpoint());
        SmartDashboard.putNumber("Arm Setpoint Error", m_positionController.getPositionError());
        SmartDashboard.putNumber("Arm Speed", m_armMotor.get());
        SmartDashboard.putBoolean("Arm Is At Setpoint", getIsAtSetpoint());
    }
}
