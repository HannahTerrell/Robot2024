package frc.robot.commands.Autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.XCaliper;
import frc.robot.commands.*;

public class DriveAuton extends SequentialCommandGroup {
    public DriveAuton(Drivetrain drivetrain, AHRS gyro, XCaliper robot) {
        addRequirements(drivetrain);
        addCommands(
            new DriveDistance(2.0, drivetrain, gyro, robot) 
        );
    }
}
