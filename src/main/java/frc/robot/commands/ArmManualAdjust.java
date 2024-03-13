package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controls;
import frc.robot.subsystems.Arm;

public class ArmManualAdjust extends Command {
    private Arm arm;
    private Controls controllerProfile;

    public ArmManualAdjust(Arm arm, Controls controls) {
        super();
        addRequirements(arm);
        this.arm = arm;
        this.controllerProfile = controls;
    }

    @Override
    public void execute() {
        arm.adjustAim(controllerProfile.getArmAdjustUp());
    }
}
