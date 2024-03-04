// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  TO-DO: 
  Change motors and encoders as necessary (NEOs drive and turn)
  Tune PID loops for drive and turn
  Tune feed forwards for drive and turn
*/
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AnalogEncoder8612;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
  private static final double kWheelRadius = 0.0508;
  private static final double kDriveGearboxRatio = 6.12;
  
  public static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogEncoder8612 m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(2.5, 0, 0,
        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.2, 0.6);
  private DoublePublisher m_turningEncoderDistancePublisher;
  private DoublePublisher m_turningEncoderVoltagePublisher;
  private DoublePublisher m_TurnPublisher;
  private DoublePublisher m_driveVelocityPublisher;
  private DoublePublisher m_driveVoltagePublisher;
  private DoublePublisher m_driveDistancePublisher;
  private DoublePublisher m_driveDesiredVelocityPublisher;
  private DoublePublisher m_driveDesiredVelocityRawPublisher;
  private DoublePublisher m_driveOutputPublisher;
  private DoublePublisher m_desiredAnglePublisher;
  private AnalogInput m_turningInput;


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannel Input for the turning encoder channel
   * @param gyroOffsetInRadians Amount to adjust the zero angle of the module by
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double gyroOffsetInRadians) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.setSmartCurrentLimit(70);
    m_turningMotor.setSmartCurrentLimit(70);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningInput = new AnalogInput(turningEncoderChannel);
    m_turningEncoder = new AnalogEncoder8612(m_turningInput);

    //var encoderResolution = m_driveEncoder.getCountsPerRevolution(); // 4096
    var driveDistancePerMotorRotation = (kWheelRadius * 2 * Math.PI) / kDriveGearboxRatio;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor((driveDistancePerMotorRotation / 1.65) * 1.575);
    m_driveEncoder.setVelocityConversionFactor(m_driveEncoder.getPositionConversionFactor() / 60);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setDistancePerRotation(Constants.Tau);

    var positionOffset = (gyroOffsetInRadians % Constants.Tau) / (Constants.Tau);
    m_turningEncoder.setPositionOffset(positionOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_turningEncoderDistancePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Turning/Distance").publish();
    
    m_turningEncoderVoltagePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Turning/Voltage").publish();

    m_TurnPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Turning/Output").publish();

    m_driveVelocityPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Velocity").publish();

    m_driveVoltagePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Voltage").publish();

    m_driveDistancePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Distance").publish();

    m_driveDesiredVelocityPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Desired Velocity").publish();

    m_driveDesiredVelocityRawPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Desired Velocity (raw)").publish();

    m_driveOutputPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Output").publish();

    m_desiredAnglePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("/SwerveModules/" + name + "/Drive/Desired Angle").publish();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    m_driveDesiredVelocityRawPublisher.set(state.speedMetersPerSecond);

    m_desiredAnglePublisher.set(state.angle.getRadians());

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    m_driveDesiredVelocityPublisher.set(state.speedMetersPerSecond);


    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    m_driveOutputPublisher.set(driveOutput);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);

    m_TurnPublisher.set(turnOutput);
    m_driveVoltagePublisher.set(driveOutput + driveFeedforward);
  }

  public double getEncoderDistance() {
    return m_driveEncoder.getPosition();
  }

  public void resetEncoderDistance() {
    m_driveEncoder.setPosition(0);
  }


  public CANSparkMax getModuleMotor() {
    return m_driveMotor;
  }

  public void stop() {
    m_driveMotor.setVoltage(0);
    m_turningMotor.setVoltage(0);
  }

  public double getTurningEncoderValue() {
    return m_turningEncoder.getAbsolutePosition();
  }


  public void periodic() {
    m_turningEncoderDistancePublisher.set(m_turningEncoder.getDistance());
    m_turningEncoderVoltagePublisher.set(m_turningInput.getVoltage());
    m_driveVelocityPublisher.set(m_driveEncoder.getVelocity());
    m_driveDistancePublisher.set(m_driveEncoder.getPosition());
  }
}
