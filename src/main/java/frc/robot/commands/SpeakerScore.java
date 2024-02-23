package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class SpeakerScore extends SequentialCommandGroup {
    public SpeakerScore(Shooter shooter, ArmUp armUp, ArmDown armDown, Limelight limelight) {
        addRequirements(shooter, limelight);
        addCommands(
            armUp,
            new RunCommand(() -> shooter.shootSpeaker()).withTimeout(1),
            armDown
        );
    }
}
