package frc.robot.subsystems.GroundIntake;
import com.ctre.phoenix6.hardware.*;


public interface GroundIntakeIO {
    public class GroundIntakeIOInputs {
        public double intakeSpeed = 0.0;
        public double encoderPosition = 0.0;
    }
    default void updateInputs(GroundIntakeIOInputs inputs) {}
    default void setPositionSetpoint(double setpoint) {}
    default void refreshData() {}
    default void setIntakeSpeed(double speed) {}
    default void setPivotSpeed(double speed) {}
    default double getEncoderVal() {return 0;}
}
