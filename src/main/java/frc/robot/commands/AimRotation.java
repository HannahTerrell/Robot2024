package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// this command is continuous. Use the Limelight.getAimedAtTargetTrigger to end the command
public class AimRotation extends Command {
    private PIDController controller;
    private Drivetrain drivetrain;
    private Limelight limelight;

    public AimRotation(Drivetrain drivetrain, Limelight limelight, PIDController rotationPidController) {
        super();
        this.controller = rotationPidController;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        controller.reset();
        drivetrain.setRotationOverrideSupplier(this::getRotationOverride);
    }

    @Override
    public void execute() {
        // work is applied as an override in the Drivetrain drive methods.
    }

    @Override
    public void end(boolean interrupted) {
        controller.reset();
        drivetrain.setRotationOverrideSupplier(null);
    }

    private Rotation2d getRotationOverride()
    {
        var targetX = limelight.getTargetX();
        if (targetX == 0) return null;

        var rot = controller.calculate(limelight.getTargetX());
        rot = MathUtil.clamp(rot, -0.5, 0.5);

        return Rotation2d.fromRadians(rot * drivetrain.getMaximumAngularVelocity());
    }
}
