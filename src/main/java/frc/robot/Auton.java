package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AimArm;
import frc.robot.commands.ArmDown;
import frc.robot.commands.FeedAndShoot;
import frc.robot.commands.IntakeAndFeed;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.StopSystems;
import frc.robot.subsystems.Subsystems;

public class Auton {
    public Auton(
        Subsystems subsystems,
        ShootSpeaker shootSpeaker
    ) {
        super();

        NamedCommands.registerCommand("stopModules", new InstantCommand(() -> {subsystems.getDrivetrain().stopModules();}));
        NamedCommands.registerCommand("stopSystems", new StopSystems(subsystems.getShooter(), subsystems.getIntake(), subsystems.getArm()));
        NamedCommands.registerCommand("shootSpeaker", shootSpeaker);
        NamedCommands.registerCommand("shootAmp", new RunCommand(() -> {subsystems.getShooter().shootAmp();}).withTimeout(1.0));
        NamedCommands.registerCommand("aimArm", new AimArm(subsystems.getArm(), subsystems.getLimelight(), false));
        NamedCommands.registerCommand("armDown", new ArmDown(subsystems.getArm()));
        NamedCommands.registerCommand("intakeAndFeed", new IntakeAndFeed(subsystems.getShooter(), subsystems.getIntake()));
        NamedCommands.registerCommand("feedAndShoot", new FeedAndShoot(subsystems.getShooter(), subsystems.getIntake()));
    }

    public SendableChooser<Command> getAutonChooser() {

        var autonChooser = AutoBuilder.buildAutoChooser("Two-Speaker Auto");
        
        return autonChooser;
    }
}
