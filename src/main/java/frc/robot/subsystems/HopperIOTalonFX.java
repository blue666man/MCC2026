package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX beltMotor;

public HopperIOTalonFX(int beltID) {
        beltMotor = new TalonFX(beltID);
}

   @Override
    public void setBeltSpeed(double speed) {

    }

  @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.beltSpeed = beltMotor.get();
    }

    @Override
    public void refreshData() {
        // Not required for Spark MAX, but useful for manual telemetry push or debug logging
        SmartDashboard.putNumber("Hopper Speed", beltMotor.get());
    }


}