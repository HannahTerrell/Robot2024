package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> getForwardAxis;
    private Supplier<Double> getLeftAxis;
    private Supplier<Double> getRotationCCWAxis;

    public TeleopDrive(
            Drivetrain drivetrain,
            Supplier<Double> getForwardAxis,
            Supplier<Double> getLeftAxis,
            Supplier<Double> getRotationCCWAxis) {
        super();
        this.drivetrain = drivetrain;
        this.getForwardAxis = getForwardAxis;
        this.getLeftAxis = getLeftAxis;
        this.getRotationCCWAxis = getRotationCCWAxis;
    }

    @Override
    public void execute() {
        var cs = new ChassisSpeeds(
                drivetrain.getMaximumVelocity() * getForwardAxis.get(),
                drivetrain.getMaximumVelocity() * getLeftAxis.get(),
                drivetrain.getMaximumAngularVelocity() * getRotationCCWAxis.get());

        drivetrain.driveFieldRelative(cs);
    }
}
