package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class AmpScore extends SequentialCommandGroup {
    public AmpScore(Shooter shooter, ArmUp armUp, ArmDown armDown) {
        addRequirements(shooter);
        addCommands(
            armUp,
            new RunCommand(() -> {shooter.shootAmp();}).withTimeout(1),
            armDown
        );
    }
}
