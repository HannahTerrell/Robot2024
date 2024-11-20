package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;

public class NavX extends AHRS {
    public NavX(Port serial_port_id) {
        super(serial_port_id);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("TempC", this::getTempC, null);

        // Angle
		builder.publishConstString("Angle/.type", "Gyro");
        builder.addDoubleProperty("Angle/Value", this::getAngle, null);
        builder.addDoubleProperty("Angle/Adjustment", this::getAngleAdjustment, this::setAngleAdjustment);
        builder.addDoubleProperty("Angle/Rate", this::getRate, null);
        
        // Compass
		builder.publishConstString("Compass/.type", "Gyro");
        builder.addDoubleProperty("Compass/Value", this::getCompassHeading, null);
        
        // FusedHeading
		builder.publishConstString("FusedHeading/.type", "Gyro");
        builder.addDoubleProperty("FusedHeading/Value", this::getFusedHeading, null);

        // Yaw
		builder.publishConstString("Yaw/.type", "Gyro");
        builder.addDoubleProperty("Yaw/Value", this::getYaw, null);

        // Pitch
        builder.addDoubleProperty("Pitch/Value", this::getPitch, null);
        
        // Roll
        builder.addDoubleProperty("Roll/Value", this::getRoll, null);

        // Accel
		builder.publishConstString("Accel/.type", "3AxisAccelerometer");
        builder.addDoubleProperty("Accel/X", this::getWorldLinearAccelX, null);
        builder.addDoubleProperty("Accel/Y", this::getWorldLinearAccelY, null);
        builder.addDoubleProperty("Accel/Z", this::getWorldLinearAccelZ, null);
    }
}
