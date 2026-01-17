package frc.robot.subsystems.Shooter;

public interface ShooterIO {
     default void setShooterSpeed (double speed) {}

      public class shooterIOInputs {
        public double shooterSpeed = 0.0;
    }
    default void updateInputs(shooterIOInputs inputs) {}
    default void refreshData() {}
}


