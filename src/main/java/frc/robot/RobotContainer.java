// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.controllers.Controls;
import frc.robot.controllers.DualXboxControllers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link XCaliper}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final Drivetrain m_swerve = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();
  private final Limelight m_limelight = new Limelight();
  private final Subsystems m_subsystems = new Subsystems(
    m_swerve, 
    m_intake,
    m_arm, 
    m_shooter, 
    m_climber, 
    m_limelight
  );

  //Commands
  private final ShootSpeaker shootSpeaker = new ShootSpeaker(m_shooter);

  //Auton chooser initiation
  private final SendableChooser<Command> m_autonChooser;

  //Controls
  private final Controls m_controls = new DualXboxControllers();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    var auton = new Auton(m_subsystems, shootSpeaker);
    m_autonChooser = auton.getAutonChooser();

    //Auton chooser
    SmartDashboard.putData("Auton Chooser", m_autonChooser);
  }

  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }

  public void autonomousPeriodic() {
  }

  public void teleopInit() {
    m_intake.setDefaultCommand(new IntakeManual(m_intake, m_controls));
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, m_controls));
    m_arm.setDefaultCommand(new ArmManualAdjust(m_arm, m_controls));
    m_shooter.setDefaultCommand(new RunCommand(() -> {}, m_shooter));

    m_controls.configureBindings(
      m_subsystems,
      shootSpeaker
    );
  }

  public void teleopPeriodic() {
  }


  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_limelight.periodic();
    SmartDashboard.putBoolean("Have Note?", !m_intake.hasNote());

    SmartDashboard.putData(CommandScheduler.getInstance());
  }
}
