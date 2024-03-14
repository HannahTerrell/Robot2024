package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RotationAimController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AimRotation extends Command {
    private RotationAimController controller;
    private Drivetrain drivetrain;
    private Limelight limelight; 

    public AimRotation(Drivetrain drivetrain, Limelight limelight) {
        super();
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
        controller = new RotationAimController(limelight);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        drivetrain.drive(0, 0, controller.calculate());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(limelight.getTargetX()) < 3; 
    }
}
