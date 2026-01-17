package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable table;
    public Limelight(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public boolean isSeeingValidTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }
}
