package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;

public abstract interface Controls {
    public abstract Command getTeleopDriveCommand();
}
