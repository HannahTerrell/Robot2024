package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controls;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {
    protected Drivetrain drivetrain;
    protected Controls controls;

    public TeleopDrive(Drivetrain drivetrain, Controls controls) {
        super();
        this.drivetrain = drivetrain;
        this.controls = controls;
        addRequirements(this.drivetrain);
    }

    protected double getRotationLeft() {
        return Math.pow(controls.getRobotRotationLeft(), 3);
    }

    @Override
    public void execute() {
        drivetrain.drive(
            Math.pow(controls.getRobotForward(), 3),
            Math.pow(controls.getRobotStrafeLeft(), 3),
            getRotationLeft()
        );
    }
}
