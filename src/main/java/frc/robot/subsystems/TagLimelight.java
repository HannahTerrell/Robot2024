package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TargetType;

public class TagLimelight extends Limelight {
    private int m_lastAprilTag = 0;
    private NetworkTableEntry tid;

    public TagLimelight() {
        super("limelight-tags");

        tid = table.getEntry("tid");
    }

    public int getAprilTag() {
        return (int)tid.getInteger(0);
    }

    public int getLastAprilTag() {
        return m_lastAprilTag;
    }

    public TargetType getLastTargetType() {
        switch (m_lastAprilTag) {
            case 5:
            case 6:
                return TargetType.AMP;

            case 4:
            case 7:
                return TargetType.SPEAKER;

            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
                return TargetType.STAGE;
    
            default:
                return TargetType.NONE;
        }
    }

    public void periodic() {
        var aprilTag = getAprilTag();
        if (aprilTag != 0) {
            m_lastAprilTag = aprilTag;
        }

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);

        //post to smart dashboard
        SmartDashboard.putNumber("Tag Limelight X", x);
        SmartDashboard.putNumber("Tag Limelight Y", y);
        SmartDashboard.putNumber("Tag Limelight ID", getAprilTag());
        SmartDashboard.putString("Tag Limelight LastTargetType", getLastTargetType().toString());
    }
}
