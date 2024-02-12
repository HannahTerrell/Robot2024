package frc.robot.commands.Autonomous;

import java.util.List;
import java.util.function.Consumer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class PathAuton1 extends SequentialCommandGroup {
    private Drivetrain m_drivetrain;
    Trajectory trajectory;
    TrajectoryConfig trajectoryConfig;
    private PIDController xController = new PIDController(1, 0, 0);
    private PIDController yController = new PIDController(1, 0, 0); 
    private ProfiledPIDController rotController = new ProfiledPIDController(2.5, 0, 0,
        new TrapezoidProfile.Constraints(SwerveModule.kModuleMaxAngularVelocity, SwerveModule.kModuleMaxAngularAcceleration));
    SwerveControllerCommand swerveControllerCommand;

    public PathAuton1(Drivetrain drivetrain) {
        super();
        m_drivetrain = drivetrain;
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        trajectoryConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed, 1)
            .setKinematics(m_drivetrain.getKinematics());

        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ), 
            new Pose2d(3, 0, new Rotation2d(0)),
            trajectoryConfig
        );

        swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            m_drivetrain::getPose, 
            m_drivetrain.getKinematics(), 
            xController,
            yController,
            rotController,
            m_drivetrain::setModuleStates, 
            m_drivetrain
        );

        addCommands(
            new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> m_drivetrain.stopModules())
        );
    }

}
