package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class Limelight {
    protected NetworkTable table;
    protected NetworkTableEntry tx;
    protected NetworkTableEntry ty;
    protected String name;

    public Limelight(String name) {
        super();
        this.name = name;

        table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
    }

    public double getTargetX() {
        return tx.getDouble(0);
    }

    public double getTargetY() {
        return ty.getDouble(0);
    }

    public boolean isAimedAtTarget() {
        return Math.abs(getTargetX()) < 5;
    }
}
