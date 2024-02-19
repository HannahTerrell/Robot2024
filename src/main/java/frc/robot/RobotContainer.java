// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
//import frc.robot.commands.*;
import frc.robot.commands.Autonomous.*;
//import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link XCaliper}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private XCaliper m_robot;

  //Subsystems
  private final Drivetrain m_swerve = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();
  private final Limelight m_limelight = new Limelight();

  //Commands
  //private final AmpScore m_ampScoreCommand = new AmpScore(m_shooter, m_arm);

  //Electronics
  //private AHRS m_gyro = m_swerve.getGyro();
  // private final PowerDistribution m_powerDistribution = new PowerDistribution();
  
  //Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  //Controllers
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  //Buttons and axes
  private final int m_intakeAxis = XboxController.Axis.kLeftY.value;
  private final JoystickButton m_climbUpButton = new JoystickButton(m_operatorController, 2);
  private final JoystickButton m_climbDownButton = new JoystickButton(m_operatorController, 0);
  private final JoystickButton m_shootSpeakerButton = new JoystickButton(m_operatorController, 3);
  private final JoystickButton m_shootAmpButton = new JoystickButton(m_operatorController, 4);
  private final JoystickButton m_aimButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

  //Auton things
  private final PathAuton1 m_pathAuton1 = new PathAuton1(m_swerve);
  private final PathAuton2 m_pathAuton2 = new PathAuton2(m_swerve);
  private final PathAuton3 m_pathAuton3 = new PathAuton3(m_swerve);
  SendableChooser<Command> m_autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(XCaliper robot) {
    // Configure the trigger bindings
    configureBindings();
    m_robot = robot;

    //Adding auton routines
    m_autonChooser.setDefaultOption("One-Note Auton", m_pathAuton1);
    m_autonChooser.addOption("Two-Note Auton", m_pathAuton2);
    m_autonChooser.addOption("Drive Auton", m_pathAuton3);
    SmartDashboard.putData("Auton Chooser", m_autonChooser);
  }

  public void teleopInit() {
    m_intake.setDefaultCommand(
      new RunCommand(() -> {
        m_intake.setSpeed(-(m_operatorController.getRawAxis(m_intakeAxis)));
        m_intake.feed();
      },
      m_intake));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_climbUpButton.onTrue(new RunCommand(() -> {
      m_climber.climbUp();
    }));

    m_climbDownButton.onTrue(new RunCommand(() -> {
      m_climber.climbDown();
    }));

    m_shootAmpButton.onTrue(new RunCommand(() -> {
      m_arm.moveToTag(m_limelight);
      m_shooter.shootAmp();
    }));

    m_shootSpeakerButton.onTrue(new RunCommand(() -> {
      m_arm.moveToTag(m_limelight);
      m_shooter.shootSpeaker();
    }));
  }

  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.2)) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.2)) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
       -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.2)) * Drivetrain.kMaxAngularSpeed;

    double limelight_tx = m_limelight.getTX().getDouble(0);

    if (m_aimButton.getAsBoolean() && limelight_tx != 0 && Math.abs(limelight_tx) > 2) {
      // xSpeed = m_xspeedLimiter.calculate(1) * Drivetrain.kMaxSpeed;
      // ySpeed = m_yspeedLimiter.calculate(1) * Drivetrain.kMaxSpeed;
        rot = -limelight_tx * 0.02 * Drivetrain.kMaxAngularSpeed;
        m_arm.moveToTag(m_limelight);
    }
    
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, m_robot.getPeriod());
  }

  /**
   * Use this to pass the autonomous command to the main {@link XCaliper} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_limelight.periodic();
  }
}
