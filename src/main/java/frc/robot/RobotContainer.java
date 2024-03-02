// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final ArmUp armUp = new ArmUp(m_arm);
  private final ArmDown armDown = new ArmDown(m_arm);
  private final FeedAndShoot feedAndShoot = new FeedAndShoot(m_shooter, m_intake);
  private final IntakeAndFeed intakeAndFeed = new IntakeAndFeed(m_shooter, m_intake);
  private final StopSystems stopSystems = new StopSystems(m_shooter, m_intake, m_arm);
  //private final AmpScore ampScore = new AmpScore(m_shooter, armUp, armDown);

  //Auton chooser initiation
  SendableChooser<Command> m_autonChooser;
  
  //Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  //Controllers
  //private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  //Buttons and axes
  // private final int m_intakeAxis = XboxController.Axis.kLeftY.value;
  private final int m_armAxis = XboxController.Axis.kRightY.value;
  private final JoystickButton m_climbUpButton = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_climbDownButton = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
  private final POVButton m_leftClimbAdjust = new POVButton(m_operatorController, 90);
  private final POVButton m_rightClimbAdjust = new POVButton(m_operatorController, 270);
  private final JoystickButton m_shootSpeakerButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  private final JoystickButton m_shootAmpButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  private final JoystickButton m_feedOnlyButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  private final JoystickButton m_shootOnlyButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  private final JoystickButton m_backfeedButton = new JoystickButton(m_operatorController, 8);
  private final JoystickButton m_aimButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_resetFieldRelativeButton = new JoystickButton(m_driverController, 7);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(XCaliper robot) {
    // Configure the trigger bindings
    configureBindings();

    m_robot = robot;

    //Commands for PathPlanner
    NamedCommands.registerCommand("stopModules", new InstantCommand(() -> {m_swerve.stopModules();}));
    NamedCommands.registerCommand("stopSystems", stopSystems);
    NamedCommands.registerCommand("shootSpeaker", new RunCommand(() -> {m_shooter.shootSpeaker();}).withTimeout(1.0));
    NamedCommands.registerCommand("shootAmp", new RunCommand(() -> {m_shooter.shootAmp();}).withTimeout(1.0));
    NamedCommands.registerCommand("armUp", armUp);
    NamedCommands.registerCommand("armDown", armDown);
    NamedCommands.registerCommand("intakeAndFeed", intakeAndFeed);
    NamedCommands.registerCommand("feedAndShoot", feedAndShoot);

    //Auton things
    final PathPlannerAuto m_pathplanner1 = new PathPlannerAuto("One-Amp Auto");
    final PathPlannerAuto m_pathplanner2 = new PathPlannerAuto("Two-Amp Auto");
    final PathPlannerAuto m_pathplanner3 = new PathPlannerAuto("Two-Speaker Auto");
    final PathPlannerAuto m_pathplanner4 = new PathPlannerAuto("Demo Auto");
    final PathPlannerAuto m_pathplanner5 = new PathPlannerAuto("Speaker-Podium Auto (Non-Amp)");
    final PathPlannerAuto m_pathplanner6 = new PathPlannerAuto("Speaker-Podium Auto (Center)");
    final PathPlannerAuto m_pathplanner7 = new PathPlannerAuto("Disruption Auto");
    final PathPlannerAuto m_pathplanner8 = new PathPlannerAuto("Test Auto");

    //Auton chooser
    m_autonChooser = new SendableChooser<>();

    //Adding auton routines to chooser
    m_autonChooser.addOption("One-Amp Auton", m_pathplanner1);
    m_autonChooser.addOption("Two-Amp Auton", m_pathplanner2);
    m_autonChooser.addOption("Two-Speaker Auton", m_pathplanner3);
    m_autonChooser.addOption("Demo Auton", m_pathplanner4);
    m_autonChooser.addOption("Speaker-Podium Auton (Non-Amp)", m_pathplanner5);
    m_autonChooser.addOption("Speaker-Podium Auton (Center)", m_pathplanner6);
    m_autonChooser.addOption("Disruption Auton", m_pathplanner7);
    m_autonChooser.addOption("Test Auton", m_pathplanner8);
    SmartDashboard.putData("Auton Chooser", m_autonChooser);
  }

  public void teleopInit() {
    m_intake.setDefaultCommand(
      new RunCommand(() -> {
        var intakeSpeed = (m_operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value)
           - m_operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value));
        // var intakeSpeed = -(m_operatorController.getRawAxis(m_intakeAxis));
        m_intake.intake(intakeSpeed);
      },
      m_intake));

      m_arm.setDefaultCommand(
      new RunCommand(() -> {
        m_arm.setSpeed(-m_operatorController.getRawAxis(m_armAxis));
      },
      m_arm));
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
    m_climbUpButton.whileTrue(new StartEndCommand(() -> {
      m_climber.climbUp(-1);
    },
    () -> {
      m_climber.stop();
    }, m_climber));

    m_climbDownButton.whileTrue(new StartEndCommand(() -> {
      m_climber.climbDown(1);
    },
    () -> {
      m_climber.stop();
    }, m_climber));

    m_leftClimbAdjust.whileTrue(new StartEndCommand(() -> {
      m_climber.adjustSide("left");
    },
    () -> {
      m_climber.stop();
    }, m_climber));

    m_rightClimbAdjust.whileTrue(new StartEndCommand(() -> {
      m_climber.adjustSide("right");
    },
    () -> {
      m_climber.stop();
    }, m_climber));

    m_shootAmpButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.shootAmp();
    },
    () -> {
      m_shooter.stop();
    }));

    m_shootSpeakerButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.shootSpeaker();
    },
    () -> {
      m_shooter.stop();
    }));

    m_feedOnlyButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.feedOnly();
    },
    () -> {
      m_shooter.stop();
    }));

    m_shootOnlyButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.shootSpeakerOnly();
    },
    () -> {
      m_shooter.stop();
    }));

     m_backfeedButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.backfeed();
    },
    () -> {
      m_shooter.stop();
    }));

    m_resetFieldRelativeButton.onTrue(new InstantCommand(() -> {
      m_swerve.resetFieldRelative();
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
    // var rotWODeadband = m_driverController.getRightTriggerAxis() 
    //   - m_driverController.getLeftTriggerAxis();
    var rotWODeadband = m_driverController.getRightX();
    var rot =
       -m_rotLimiter.calculate(MathUtil.applyDeadband(rotWODeadband, 0.2)) * Drivetrain.kMaxAngularSpeed;

    double limelight_tx = m_limelight.getTX().getDouble(0);

    if (m_aimButton.getAsBoolean() && limelight_tx != 0 && Math.abs(limelight_tx) > 2) {
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

    SmartDashboard.putData(CommandScheduler.getInstance());
  }
}
