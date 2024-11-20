// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.controls.Controls;
import frc.robot.controls.DefaultControls;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.NavX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link XCaliper}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Limelight m_tagsLimelight = new Limelight("limelight-tags",
      new Pose3d(.28, 0, .2, new Rotation3d(0, 15, 0)));

  // Controls
  private final Controls m_controls = new DefaultControls(m_drivetrain);

  // State
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  private final NavX navx = new NavX(Port.kMXP);
  private final SwerveDrivePoseEstimator poseEstimator = m_drivetrain.swerveDrivePoseEstimator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_controls.getTeleopDriveCommand());

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Drivetrain", m_drivetrain);
    SmartDashboard.putData("PDP", new PowerDistribution());
    SmartDashboard.putData("NavX", navx);
    SmartDashboard.putData(accelerometer);

    // Field already in smart dashboard by YAGSL telemetry
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    updateVisionPoseTracking();
  }

  private void updateVisionPoseTracking() {
    // if rotating quickly, don't try to use the limelight tracking
    if (navx.getRate() > 720) {
      return;
    }

    var robotOrientation = poseEstimator.getEstimatedPosition().getRotation();
    var poseEstimate = m_tagsLimelight.getBotPoseEstimate_wpiBlue_MegaTag2(robotOrientation);

    if (poseEstimate.tagCount == 0) {
      return;
    }

    poseEstimator.addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(.6, .6, 9999999)); // there is some fancy math here I don't understand, but this is what online examples use
  }
}
