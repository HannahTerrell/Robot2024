package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class SpeakerScore extends SequentialCommandGroup {
    public SpeakerScore(Shooter shooter, Arm arm, Limelight limelight) {
        addRequirements(shooter, arm, limelight);
        addCommands(
            new RunCommand(() -> arm.moveToTag(limelight)),
            new RunCommand(() -> shooter.shootSpeaker()).withTimeout(1),
            new RunCommand(() -> arm.moveDown())
        );
    }
}
