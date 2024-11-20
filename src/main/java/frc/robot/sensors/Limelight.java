package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight {
    private String name;

    public Limelight(String name, Pose3d cameraPoseInRobotSpace) {
        super();
        this.name = name;

        LimelightHelpers.setCameraPose_RobotSpace(name, 0, 0, 0, 0, 0, 0);
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            cameraPoseInRobotSpace.getX(),
            cameraPoseInRobotSpace.getY(),
            cameraPoseInRobotSpace.getZ(),
            cameraPoseInRobotSpace.getRotation().getX(),
            cameraPoseInRobotSpace.getRotation().getY(),
            cameraPoseInRobotSpace.getRotation().getZ()
        );
    }

    public boolean isTargetValid() {
        return LimelightHelpers.getTV(name);
    }

    public Measure<Angle> getTargetXAngle() {
        return Degrees.of(LimelightHelpers.getTX(name));
    }

    public Measure<Angle> getTargetYAngle() {
        return Degrees.of(LimelightHelpers.getTY(name));
    }

    public Pose3d getTargetPose() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(name);
    }

    public PoseEstimate getBotPoseEstimate_wpiBlue() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(Rotation2d robotOrientation) {
        LimelightHelpers.SetRobotOrientation(name, robotOrientation.getDegrees(), 0, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }
}
