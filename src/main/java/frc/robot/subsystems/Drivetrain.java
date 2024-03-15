// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 4.0; // 4 meters per second
  public static final double kMaxAngularSpeed = 1.5 * Math.PI; // 1.5 rotations per second

  //Swerve drive object.
  private final SwerveDrive swerveDrive;

  //Maximum speed of the robot in meters per second, used to limit acceleration.
  public double maximumSpeed = Units.feetToMeters(14.5);

  public Drivetrain() {
    var directory = new File(Filesystem.getDeployDirectory(), "swerve");

    System.out.println("Drivetrain");
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    setMotorBrake(true);

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getCurrentSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        AutonConstants.TRANSLATION_PID,
        AutonConstants.ANGLE_PID,
        kMaxSpeed, 
        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    swerveDrive.drive(
      new Translation2d(
        xSpeed * swerveDrive.getMaximumVelocity(),
        ySpeed * swerveDrive.getMaximumVelocity()
      ),
      rot * swerveDrive.getMaximumAngularVelocity(),
      true,
      false
    );
  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeed) {
    swerveDrive.driveFieldOriented(chassisSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeed) {
    swerveDrive.drive(chassisSpeed);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return swerveDrive.getFieldVelocity();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void stopModules() {
  }

  public void resetFieldRelative() {
    resetPose(new Pose2d());
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  @Override
  public void periodic() {
    swerveDrive.getModules()[0].getAngleMotor().getPosition();
  }
}
 