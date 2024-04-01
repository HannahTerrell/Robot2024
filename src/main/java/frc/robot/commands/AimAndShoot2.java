package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagLimelight;

/**
 * Command to begin:
 * - Aiming arm
 * - Aiming rotation
 * - Spinning up shooter
 * 
 * The command waits for the arm to be aimed, the drive rotation to be minimal, 
 * and the limelight to be aimed, and then shoots.
 * 
 * The idea is that this command can be run while driving a path.
 */
public class AimAndShoot2 extends SequentialCommandGroup {
    private Arm arm;
    private TagLimelight tagLimelight;
    private Drivetrain drivetrain;

    public AimAndShoot2(Arm arm, Drivetrain drivetrain, TagLimelight tagLimelight, AimRotation aimRotation, AimArm aimArm, ShooterSpinUp shooterSpinUp, ShootSpeaker shootSpeaker) {
        super();
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.tagLimelight = tagLimelight;
        addCommands(
            new ParallelDeadlineGroup(
                new WaitUntilCommand(this::isReadyToShoot)
                    .andThen(shootSpeaker),
                aimRotation,
                aimArm,
                shooterSpinUp
            )
        );
    }

    private boolean isReadyToShoot() {
        return 
            arm.getIsAtSetpoint() 
            && tagLimelight.isAimedAtTarget()
            && drivetrain.getCurrentSpeeds().omegaRadiansPerSecond < 0.1;
    }
}