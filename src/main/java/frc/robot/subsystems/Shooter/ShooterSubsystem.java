package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIO.shooterIOInputs inputs = new ShooterIO.shooterIOInputs();

    public enum WantedState {
        IDLE,
        SHOOTING,
        REVERSE
    }

    private enum SystemState {
        IDLED,
        SHOOTING,
        REVERSING
    }
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;

     private double feederSpeedsetPoint = 0.0;
     private double shooterSpeedPoint = 0.0;

     public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

     @Override
    public void periodic() {
        io.updateInputs(inputs);

        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            systemState = newState;
        }
    
    


    switch (systemState) {
            case SHOOTING:
                io.setFeederSpeed(0.1);
                io.setShooterSpeed(shooterSpeedPoint);
                break;
            case REVERSING:
                io.setFeederSpeed(-feederSpeedsetPoint);
                io.setFeederSpeed(-shooterSpeedPoint);
                break;
            case IDLED:
            default:
                io.setFeederSpeed(0.0);
                io.setShooterSpeed(0.0);
                break;
        }
    }

      private SystemState handleStateTransition() {
        switch (wantedState) {
            case SHOOTING:
                return SystemState.SHOOTING;
            case REVERSE:
                return SystemState.REVERSING;
            case IDLE:
            default:
                return SystemState.IDLED;
        }
    }
    
    public void feed(double feederSpeed, double shooterSpeed) {
        this.feederSpeedsetPoint = feederSpeed;
        this.shooterSpeedPoint = shooterSpeed;
        setWantedState(WantedState.SHOOTING);
    }
     public void reverse(double feederSpeed, double shooterSpeed) {
        this.feederSpeedsetPoint = feederSpeed;
        this.shooterSpeedPoint = shooterSpeed;

        setWantedState(WantedState.REVERSE);
    }
      public void stop() {
        setWantedState(WantedState.IDLE);
    }
 public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }
    public boolean isJammed() {
        // Replace with your actual sensor logic or return false to test
        return false;
    }
    
    public boolean hasBall() {
        // Replace with sensor or switch input
        return true;
    }
    public Command setWantedStateCommand(WantedState state) {
        return new InstantCommand(() -> setWantedState(state));

    }

   public Command setSpeedCommand(double shooterSpeed) {
    return new InstantCommand(() -> {
        this.shooterSpeedPoint = shooterSpeed;

        if (shooterSpeed > 0) {
            setWantedState(WantedState.SHOOTING);
        } else if (shooterSpeed < 0) {
            setWantedState(WantedState.REVERSE);
        } else {
            setWantedState(WantedState.IDLE);
        }
    });
}
}
    








