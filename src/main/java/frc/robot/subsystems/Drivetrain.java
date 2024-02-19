// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 5.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule("frontLeft", 1, 2, 0, 0.8);
  private final SwerveModule m_frontRight = new SwerveModule("frontRight", 3, 4, 1, 0.3);
  private final SwerveModule m_backLeft = new SwerveModule("backLeft", 7, 8, 3, 0.95);
  private final SwerveModule m_backRight = new SwerveModule("backRight", 5, 6, 2, 0.3);

  private final AHRS m_gyro = new AHRS(Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  private StructArrayPublisher<SwerveModuleState> m_SwerveStatePublisher;
  private StructPublisher<ChassisSpeeds> m_ChassisSpeedPublisher;

  public Drivetrain() {
    m_gyro.reset();

    m_SwerveStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    m_ChassisSpeedPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/ChassisSpeeds", ChassisSpeeds.struct).publish();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var chassisSpeed = ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds);

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    setModuleStates(swerveModuleStates);
    m_SwerveStatePublisher.set(swerveModuleStates);
    m_ChassisSpeedPublisher.set(chassisSpeed);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public double getLeftDistanceMeters() {
    return m_frontLeft.getEncoderDistance();
  }

  public double getRightDistanceMeters() {
    return m_frontRight.getEncoderDistance();
  } 

  public void setOutputVolts(double left, double right) {
    var rightSetpoint = right / 12;
    var leftSetpoint = left / 12;

    SmartDashboard.putNumber("Left Side Setpoint (in)", leftSetpoint);
    SmartDashboard.putNumber("Right Side Setpoint (in)", rightSetpoint);

    m_frontLeft.getModuleMotor().set(leftSetpoint);
    m_backLeft.getModuleMotor().set(leftSetpoint);
    m_frontRight.getModuleMotor().set(rightSetpoint);
    m_backRight.getModuleMotor().set(rightSetpoint);
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        }, pose);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SwerveModule getModule(int index) {
    SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
    return modules[index];
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[3];
    states[0] = getModule(0).getState();
    states[1] = getModule(1).getState();
    states[2] = getModule(2).getState();
    states[3] = getModule(3).getState();
    return states;
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putString("Robot Position", getPose().getTranslation().toString());
  }
}
 