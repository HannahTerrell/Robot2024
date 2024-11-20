package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivetrain;

public class PathPlanner {
    public static void initialize(Drivetrain drivetrain) {
        AutoBuilder.configureHolonomic(
                drivetrain::getPose,
                drivetrain::resetOdometry,
                drivetrain::getFieldVelocity,
                drivetrain::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        AutonConstants.TRANSLATION_PID,
                        AutonConstants.ANGLE_PID,
                        drivetrain.getMaximumVelocity(),
                        drivetrain.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain);

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            drivetrain.field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            drivetrain.field.getObject("path").setPoses(poses);
        });
    }
}
