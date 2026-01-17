package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIO.shooterIOInputs inputs = new ShooterIO.shooterIOInputs();

    public enum WantedState {
        IDLE,
        FEEDING,
        REVERSE
    }

    private enum SystemState {
        IDLED,
        FEEDING,
        REVERSING
    }
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;

     private double shooterSpeedSetpoint = 0.0;

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
            case FEEDING:
                io.setShooterSpeed(0.1);
                break;
            case REVERSING:
                io.setShooterSpeed(-shooterSpeedSetpoint);
                break;
            case IDLED:
            default:
                io.setShooterSpeed(0.0);
                break;
        }
    }

      private SystemState handleStateTransition() {
        switch (wantedState) {
            case FEEDING:
                return SystemState.FEEDING;
            case REVERSE:
                return SystemState.REVERSING;
            case IDLE:
            default:
                return SystemState.IDLED;
        }
    }
    
    public void feed(double shooterSpeed) {
        this.shooterSpeedSetpoint = shooterSpeed;
        setWantedState(WantedState.FEEDING);
    }
     public void reverse(double shooterSpeed) {
        this.shooterSpeedSetpoint = shooterSpeed;
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
}
    








