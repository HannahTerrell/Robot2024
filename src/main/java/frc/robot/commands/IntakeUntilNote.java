package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeUntilNote extends SequentialCommandGroup {
    public IntakeUntilNote(Intake intake, Shooter shooter, Arm arm) {
        super();
        addRequirements(intake, shooter, arm);
        addCommands(
            new ArmDown(arm),
            new InstantCommand(() -> {
                intake.intake(1);
                shooter.feedOnly();
            }),
            new WaitUntilCommand(shooter::hasNote),
            new InstantCommand(() -> {
                intake.stop();
                shooter.stopFeed();
            })
        );
    }
}
