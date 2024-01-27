// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  TO-DO: 
  Make sure motor and encoder channels are correct
  Change gyro to NavX? -> Done
*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); //TODO: Adjust these?
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

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

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        m_frontLeft.getModuleMotor().setVoltage(volts.in(Volts));
        m_frontRight.getModuleMotor().setVoltage(volts.in(Volts));
        m_backLeft.getModuleMotor().setVoltage(volts.in(Volts));
        m_backRight.getModuleMotor().setVoltage(volts.in(Volts));
      }, 
      log -> {
        log.motor("frontLeftDrive")
          .voltage(m_appliedVoltage.mut_replace(m_frontLeft.getModuleMotor().get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_frontLeft.getModuleMotor().getEncoder().getPosition(), Meters))
          .linearVelocity(m_velocity.mut_replace(m_frontLeft.getModuleMotor().getEncoder().getVelocity(), MetersPerSecond));

        log.motor("frontRightDrive")
          .voltage(m_appliedVoltage.mut_replace(m_frontRight.getModuleMotor().get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_frontRight.getModuleMotor().getEncoder().getPosition(), Meters))
          .linearVelocity(m_velocity.mut_replace(m_frontRight.getModuleMotor().getEncoder().getVelocity(), MetersPerSecond));

        log.motor("backLeftDrive")
          .voltage(m_appliedVoltage.mut_replace(m_backLeft.getModuleMotor().get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_backLeft.getModuleMotor().getEncoder().getPosition(), Meters))
          .linearVelocity(m_velocity.mut_replace(m_backLeft.getModuleMotor().getEncoder().getVelocity(), MetersPerSecond));

        log.motor("backRightDrive")
          .voltage(m_appliedVoltage.mut_replace(m_backRight.getModuleMotor().get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_backRight.getModuleMotor().getEncoder().getPosition(), Meters))
          .linearVelocity(m_velocity.mut_replace(m_backRight.getModuleMotor().getEncoder().getVelocity(), MetersPerSecond));
      }, 
      this));

  public Drivetrain() {
    m_gyro.reset();
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
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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

  public double getFrontLeftDistanceMeters() {
    return m_frontLeft.getEncoderDistance();
  }

  public double getFrontRightDistanceMeters() {
    return m_frontRight.getEncoderDistance();
  } 

  public double getBackLeftDistanceMeters() {
    return m_backLeft.getEncoderDistance();
  }

  public double getBackRightDistanceMeters() {
    return m_backRight.getEncoderDistance();
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
 