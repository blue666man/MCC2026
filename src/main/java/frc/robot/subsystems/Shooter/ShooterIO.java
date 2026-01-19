package frc.robot.subsystems.Shooter;

public interface ShooterIO {
     default void setFeederSpeed (double speed) {}
     default void setShooterSpeed (double speed) {}

      public class shooterIOInputs {
        public double feederMotor = 0.0;
        public double shooterMotor = 0.0;
        public double shooterSpeed;
        public double feederSpeed;
    }
    default void updateInputs(shooterIOInputs inputs) {}
    default void refreshData() {}
}


