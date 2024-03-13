package frc.robot.controllers;

import frc.robot.commands.ShootSpeaker;
import frc.robot.subsystems.Subsystems;

public interface Controls {
    public double getRobotForward();
    public double getRobotStrafeLeft();
    public double getRobotRotationLeft();
    public double getArmAdjustUp();
    public double getIntakeIn();

    public void configureBindings(
        Subsystems subsystems,
        ShootSpeaker shootSpeaker
    );
}
