package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.XCaliper;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives a set number of meters and stops. 
 * Uses PID controllers on the drivetrain so it SHOULD correct for straightness.
 */
public class DriveDistance extends Command {
    private Drivetrain drivetrain;
    private double distanceMeters;
    private double startingRotation;
    private XCaliper m_robot;

    private PIDController speedController;
    private PIDController rotationController;
    private AHRS gyro;

    public DriveDistance(double distanceMeters, Drivetrain drivetrain, AHRS gyro, XCaliper robot) {
        super();
        this.distanceMeters = distanceMeters;
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        speedController = new PIDController(3, 0, 0);
        rotationController = new PIDController(0.03, 0, 0.00);

        m_robot = robot;

        addRequirements(drivetrain);
    }

    private double getAverageDistance() {
        return (this.drivetrain.getLeftDistanceMeters() + this.drivetrain.getRightDistanceMeters()) / 2;
    }

    @Override
    public void initialize() {
        super.initialize();

        var startingDistance = getAverageDistance();
        startingRotation = gyro.getAngle();
        speedController.setSetpoint(startingDistance + distanceMeters);
    }

    @Override
    public void execute() {
        super.execute();

        var speed = speedController.calculate(getAverageDistance()) + 0.1;
        speed = Math.min(Math.abs(speed), 0.55) * Math.signum(speed);
        var rotationError = startingRotation - gyro.getAngle();
        var rotation = rotationController.calculate(rotationError);
        rotation = Math.min(Math.abs(rotation), 0.3) * Math.signum(rotation);
        rotation = rotation * -1; // turn the opposite direction

        drivetrain.drive(0, speed, rotation, false, m_robot.getPeriod());
    }

    @Override
    public boolean isFinished() {
        double distanceToGo = Math.abs(speedController.getSetpoint() - getAverageDistance());
        System.out.println(distanceToGo);
        return distanceToGo < 0.06;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        drivetrain.setOutputVolts(0, 0);
        System.out.println("Drive Distance " + distanceMeters + " Finished");
    }
}
