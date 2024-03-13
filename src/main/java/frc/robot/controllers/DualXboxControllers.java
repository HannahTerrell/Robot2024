package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AimArm;
import frc.robot.commands.ShootSpeaker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Subsystems;

public class DualXboxControllers implements Controls {
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private double m_drivePrecisionFactor = 1;

    public DualXboxControllers() {
        super();
    }

    @Override
    public double getRobotForward() {
        return MathUtil.applyDeadband(-driverController.getLeftY() * m_drivePrecisionFactor, 0.02);
    }

    @Override
    public double getRobotStrafeLeft() {
        return MathUtil.applyDeadband(-driverController.getLeftX() * m_drivePrecisionFactor, 0.02);
    }

    @Override
    public double getRobotRotationLeft() {
        return MathUtil.applyDeadband(-driverController.getRightX() * m_drivePrecisionFactor, 0.02);
    }

    @Override
    public double getArmAdjustUp() {
        return MathUtil.applyDeadband(-operatorController.getRightY(), 0.02);
    }

    @Override
    public double getIntakeIn() {
        return operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value)
                - operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    }

    @Override
    public void configureBindings(
        Subsystems subsystems,
        ShootSpeaker shootSpeaker
    ) {
        var drivetrain = subsystems.getDrivetrain();
        var arm = subsystems.getArm();
        var limelight = subsystems.getLimelight();
        var shooter = subsystems.getShooter();

        // precision driving button
        new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new StartEndCommand(
                () -> m_drivePrecisionFactor = 0.75, 
                () -> m_drivePrecisionFactor = 1
            ));

        // aim
        new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(new AimArm(arm, limelight, true));

        // shoot speaker
        new JoystickButton(operatorController, XboxController.Button.kX.value)
            .onTrue(shootSpeaker);

        // reset field relative
        new JoystickButton(driverController, XboxController.Button.kBack.value) // 7
            .onTrue(new InstantCommand(drivetrain::resetFieldRelative, drivetrain));
        
        // shoot amp
        new JoystickButton(operatorController, XboxController.Button.kY.value)
            .whileTrue(new StartEndCommand(shooter::shootAmp, shooter::stop));
          
        // shooter feed
        new JoystickButton(operatorController, XboxController.Button.kB.value)
            .whileTrue(new StartEndCommand(shooter::feedOnly, shooter::stop));
                
        // shooter back feed
        new JoystickButton(operatorController, XboxController.Button.kStart.value) //8
            .whileTrue(new StartEndCommand(shooter::backfeed, shooter::stop));
    
        // shoot speaker
        new JoystickButton(operatorController, XboxController.Button.kA.value)
            .whileTrue(new StartEndCommand(shooter::shootSpeakerOnly, shooter::stop));

        configureClimberBindings(subsystems.getClimber());
    }

    private void configureClimberBindings(
        Climber climber
    ) {
        JoystickButton m_climbUpButton = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
        JoystickButton m_climbDownButton = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
        POVButton m_leftClimbAdjust = new POVButton(operatorController, 90);
        POVButton m_rightClimbAdjust = new POVButton(operatorController, 270);

        m_climbUpButton.whileTrue(new StartEndCommand(() -> {
            climber.climbUp(-1);
          },
          () -> {
            climber.stop();
          }, climber));
      
        m_climbDownButton.whileTrue(new StartEndCommand(() -> {
            climber.climbDown(1);
          },
          () -> {
            climber.stop();
          }, climber));
      
        m_leftClimbAdjust.whileTrue(new StartEndCommand(() -> {
            climber.adjustSide("left");
          },
          () -> {
            climber.stop();
          }, climber));
      
        m_rightClimbAdjust.whileTrue(new StartEndCommand(() -> {
            climber.adjustSide("right");
          },
          () -> {
            climber.stop();
          }, climber));    
    }
}
