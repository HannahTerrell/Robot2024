package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.Controls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TeleopDriveWithAiming extends TeleopDrive {
    private Limelight limelight;
    private PIDController rotationController = new PIDController(0.03, 0, 0);

    public TeleopDriveWithAiming(Drivetrain drivetrain, Limelight limelight, Controls controls) {
        super(drivetrain, controls);
        this.limelight = limelight;

        SmartDashboard.putData("TeleopDriveWithAiming/RotationController", rotationController);
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    protected double getRotationLeft() {
        var tx = limelight.getTargetX();
        var rot = rotationController.calculate(tx);

        SmartDashboard.putNumber("TeleopDriveWithAiming/Rotation", rot);

        return MathUtil.clamp(rot, -1, 1);
    }
}
