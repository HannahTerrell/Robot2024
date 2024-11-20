package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;

public class DefaultControls implements Controls {
    private final XboxController m_driverController = new XboxController(0);
    // private final XboxController m_operatorController = new XboxController(1);
    private Drivetrain drivetrain;

    public DefaultControls(
        Drivetrain drivetrain
    ) {
        super();
        this.drivetrain = drivetrain;
    }

    @Override
    public Command getTeleopDriveCommand() {
        return new TeleopDrive(
            drivetrain,
            () -> Math.pow(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.02), 3),
            () -> Math.pow(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.02), 3),
            () -> Math.pow(MathUtil.applyDeadband(-m_driverController.getRightX(), 0.02), 3)
        );
    }
}
