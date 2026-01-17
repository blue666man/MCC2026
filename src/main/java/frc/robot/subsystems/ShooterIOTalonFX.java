package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX shooterMotor;

public ShooterIOTalonFX(int motorID) {
        shooterMotor = new TalonFX(motorID);
}

   @Override
    public void setShooterSpeed(double speed) {

    }

  @Override
    public void updateInputs(shooterIOInputs inputs) {
        inputs.shooterSpeed = shooterMotor.get();
    }

    @Override
    public void refreshData() {
        // Not required for Spark MAX, but useful for manual telemetry push or debug logging
        SmartDashboard.putNumber("Hopper Speed", shooterMotor.get());
    }


}