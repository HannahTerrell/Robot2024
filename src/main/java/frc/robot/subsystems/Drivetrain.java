// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  private final SwerveDrive swerveDrive = createSwerveDrive();

  public final SwerveDrivePoseEstimator swerveDrivePoseEstimator = swerveDrive.swerveDrivePoseEstimator;
  public final Field2d field = swerveDrive.field;

  public Drivetrain() {
  }

  private static SwerveDrive createSwerveDrive() {
    var directory = new File(Filesystem.getDeployDirectory(), "swerve");

    try
    {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      return new SwerveParser(directory).createSwerveDrive(4.5);
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeed) {
    swerveDrive.driveFieldOriented(chassisSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeed) {
    swerveDrive.drive(chassisSpeed);
  }

  public void resetOdometry(Pose2d pose2d) {
    swerveDrive.resetOdometry(pose2d);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public double getMaximumVelocity() {
    return swerveDrive.getMaximumVelocity();
  }

  public double getMaximumAngularVelocity() {
    return swerveDrive.getMaximumAngularVelocity();
  }

  public double getDriveBaseRadiusMeters() {
    return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @Override
  public void periodic() {
  }
}
 