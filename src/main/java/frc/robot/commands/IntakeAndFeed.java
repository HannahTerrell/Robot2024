package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeAndFeed extends ParallelCommandGroup {
    public IntakeAndFeed(Shooter shooter, Intake intake) {
        addRequirements(shooter);
        addRequirements(intake);
        addCommands(
            new RunCommand(() -> {shooter.feedOnly();}),
            new RunCommand(() -> {intake.intake(0.7);})
        );
    }
}
