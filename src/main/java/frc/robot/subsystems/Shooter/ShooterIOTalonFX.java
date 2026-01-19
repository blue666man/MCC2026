package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX feederMotor;
    private final TalonFX shooterMotor;

public ShooterIOTalonFX(int motorID) {
        feederMotor = new TalonFX(motorID);
        shooterMotor = new TalonFX(motorID);
}

   @Override
    public void setFeederSpeed(double speed) {

    }

    @Override
    public void setShooterSpeed(double speed) {

    }

  @Override
    public void updateInputs(shooterIOInputs inputs) {
        inputs.feederSpeed = feederMotor.get();
        inputs.shooterSpeed = feederMotor.get();
    }

    @Override
    public void refreshData() {
        // Not required for Spark MAX, but useful for manual telemetry push or debug logging
        SmartDashboard.putNumber("Feeding Speed", feederMotor.get());
        SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
    }


}