package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class AmpScore extends SequentialCommandGroup {
    public AmpScore(Shooter shooter, Arm arm) {
        addRequirements(shooter, arm);
        addCommands(
            new RunCommand(() -> {arm.moveUp();}),
            new RunCommand(() -> {shooter.shootAmp();}).withTimeout(1),
            new RunCommand(() -> {arm.moveDown();})
        );
    }
}
