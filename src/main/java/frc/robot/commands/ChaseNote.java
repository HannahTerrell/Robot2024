package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.Shooter;

public class ChaseNote extends SequentialCommandGroup {
    public ChaseNote(Drivetrain drivetrain, Arm arm, Shooter shooter, Intake intake, NoteLimelight limelight, double forwardMetersPerSecond, PIDController rotationAimPidCommand) {
        super();
        addRequirements(drivetrain, arm, shooter, intake);
        addCommands(
            new ParallelDeadlineGroup(
                new IntakeUntilNote(intake, shooter, arm),

                new AimRotation(drivetrain, limelight, rotationAimPidCommand),
                new RunCommand(() -> { 
                    drivetrain.driveRobotRelative(new ChassisSpeeds(forwardMetersPerSecond, 0, 0));
                })
            )
        );
    }
}
