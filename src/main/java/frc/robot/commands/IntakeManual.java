package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controls;
import frc.robot.subsystems.Intake;

public class IntakeManual extends Command{
    private Intake intake;
    private Controls controls;

    public IntakeManual(Intake intake, Controls controls) {
        super();
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake(controls.getIntakeIn());
    }
}
