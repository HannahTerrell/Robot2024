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

public class PathAuton1 extends Command {
    private Drivetrain m_drivetrain;
    private Consumer<SwerveModuleState[]> stateConsumer;
    private PIDController xController = new PIDController(1, 0, 0);
    private PIDController yController = new PIDController(1, 0, 0); 
    private ProfiledPIDController rotController = new ProfiledPIDController(2.5, 0, 0,
        new TrapezoidProfile.Constraints(SwerveModule.kModuleMaxAngularVelocity, SwerveModule.kModuleMaxAngularAcceleration));

    public PathAuton1(Drivetrain drivetrain) {
        super();
        m_drivetrain = drivetrain;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed, 1)
        .setKinematics(m_drivetrain.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0, 1)
        ), 
        new Pose2d(0, 1, new Rotation2d(0)),
        trajectoryConfig
    );

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, 
        m_drivetrain::getPose, 
        m_drivetrain.getKinematics(), 
        xController,
        yController,
        rotController,
        stateConsumer, 
        m_drivetrain
    );

    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

   @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_drivetrain.stopModules();
    }

}
