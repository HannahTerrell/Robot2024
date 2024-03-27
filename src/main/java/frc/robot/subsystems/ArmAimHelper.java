package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;

public class ArmAimHelper {
    // list of observed distances and setpoints (distance isn't true distance, because my limelightY->distance formula is bad).
    // this lets us collect different distances, so we aren't stuck with a straight line
    private final ArrayList<ArmSetpoint> m_armSetpoints = new ArrayList<ArmSetpoint>() {
        {
            add(new ArmSetpoint(0, 0));
            add(new ArmSetpoint(4, 15));
        }
    };

    public double getArmSetpoint(double distance) {
        var setpoints = m_armSetpoints.stream()
                .sorted(new ArmSetpointComparator(distance)) // sort by which setpoints are closer to the distance we a looking for 
                .limit(2) // grab just the first two
                .toList();
        
        var setpoint1 = setpoints.get(0);
        var setpoint2 = setpoints.get(1);

        // our distance is what ratio between these two setpoints?
        var distRatio = (distance - setpoint1.Distance) / (setpoint2.Distance - setpoint1.Distance);

        // apply that ratio to the setpoints range
        var setpoint = setpoint1.Setpoint + ((setpoint2.Setpoint - setpoint1.Setpoint) * distRatio);

        return setpoint;
    }

    // tracks a distance/setpoint pair
    private class ArmSetpoint {
        public ArmSetpoint(double distance, double setpoint) {
            Distance = distance;
            Setpoint = setpoint;
        }

        public double Distance;
        public double Setpoint;
    }

    // compares two ArmSetpoint by how close it is to our target distance
    private class ArmSetpointComparator implements Comparator<ArmSetpoint> {
        private double distance;

        public ArmSetpointComparator(double targetDistance) {
            this.distance = targetDistance;
        }

        @Override
        public int compare(ArmSetpoint a, ArmSetpoint b) {
            var aProximity = Math.abs(a.Distance - distance);
            var bProximity = Math.abs(b.Distance - distance);
            return Double.compare(aProximity, bProximity);
        }

    }
}
