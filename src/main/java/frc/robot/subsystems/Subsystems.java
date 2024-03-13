package frc.robot.subsystems;

public class Subsystems {
    private Drivetrain drivetrain;
    private Arm arm;
    private Shooter shooter;
    private Climber climber;
    private Limelight limelight;
    private Intake intake;

    public Subsystems(
        Drivetrain drivetrain,
        Intake intake,
        Arm arm,
        Shooter shooter,
        Climber climber,
        Limelight limelight
    ) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.arm = arm;
        this.shooter = shooter;
        this.climber = climber;
        this.limelight = limelight;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Intake getIntake() {
        return intake;
    }

    public Arm getArm() {
        return arm;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Climber getClimber() {
        return climber;
    }

    public Limelight getLimelight() {
        return limelight;
    }
}
