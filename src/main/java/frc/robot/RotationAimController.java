package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.TagLimelight;

public class RotationAimController {
    private PIDController m_rotationAimController = new PIDController(0.009, 0.00001, 0.0040);
    private TagLimelight limelight;

    public RotationAimController(TagLimelight limelight) {
        super();
        this.limelight = limelight;
    }

    public void reset() {
        m_rotationAimController.reset();
    }

    public double calculate() {
        var rot = m_rotationAimController.calculate(limelight.getTargetX());
        rot = MathUtil.clamp(rot, -0.5, 0.5);

        return rot;
    }

    public PIDController getInternalController() {
        return m_rotationAimController;
    }
}
