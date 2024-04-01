// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.io.File;
import java.util.function.Supplier;

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
  private final double maximumSpeed = Units.feetToMeters(14.5);

  private Supplier<Rotation2d> rotationOverrideSupplier = null;

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
    var rotationRadsPS = rot * swerveDrive.getMaximumAngularVelocity();

    var rotationOverride = getRotationOverride();
    if (rotationOverride != null) {
      rotationRadsPS = rotationOverride.getRadians();
    }

    swerveDrive.drive(
      new Translation2d(
        xSpeed * swerveDrive.getMaximumVelocity(),
        ySpeed * swerveDrive.getMaximumVelocity()
      ),
      rotationRadsPS,
      true,
      false
    );
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeed) {
    var chasisSpeed2 = applyRotationOverrideToRobotRelativeSpeedsIfAny(chassisSpeed);

    swerveDrive.drive(chasisSpeed2);
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

  public void resetFieldRelative() {
    resetPose(new Pose2d());
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public double getMaximumAngularVelocity() {
    return swerveDrive.getMaximumAngularVelocity();
  }

  public void setRotationOverrideSupplier(Supplier<Rotation2d> rotationOverrideSupplier) {
    this.rotationOverrideSupplier = rotationOverrideSupplier;
  }

  private Rotation2d getRotationOverride() {
    if (rotationOverrideSupplier == null) return null;

    return rotationOverrideSupplier.get();
  }

  // need to test to make sure this is right.
  // we need to be able to intercept autonomous drive ChassisSpeeds, and apply our own rotation.
  // appling our own rotation to a robot-relative ChassisSpeeds means needing to change the X and Y speed components as well, I think.
  private ChassisSpeeds applyRotationOverrideToRobotRelativeSpeedsIfAny(ChassisSpeeds chassisSpeeds) {
    var rotationOverride = getRotationOverride();
    if (rotationOverride == null) return chassisSpeeds;

    var rotated =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(rotationOverride);

    return new ChassisSpeeds(rotated.getX(), rotated.getY(), rotationOverride.getRadians());
  }

  @Override
  public void periodic() {
  }
}
 