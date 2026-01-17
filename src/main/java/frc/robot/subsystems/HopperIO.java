package frc.robot.subsystems;

public interface HopperIO {
    default void setBeltSpeed (double speed) {}
    
    public class HopperIOInputs {
        public double beltSpeed = 0.0;
    }

    default void updateInputs(HopperIOInputs inputs) {}
    default void refreshData() {}

}

 