package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix6.hardware.*;








// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {
  private static final double MIN_WHEEL_SPEED = 0;
  private static final double MIN_PIVOT_SPEED = 0;
  private final GroundIntakeIO io;
  private final GroundIntakeIO.GroundIntakeIOInputs inputs = new GroundIntakeIO.GroundIntakeIOInputs();
  double setpoint;

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

  private double wheelSpeedSetpoint;
  private double pivotSpeedSetpoint;

  public GroundIntakeSubsystem(GroundIntakeIO io, double initalWheelSpeed, double initialPivotSpeed) {
    this.io = io;
    this.wheelSpeedSetpoint = initalWheelSpeed;
    this.pivotSpeedSetpoint = initialPivotSpeed;
}

  public double getWheelSpeed() { return wheelSpeedSetpoint;}
  private void setWheelSpeed(double newSpeed) {this.wheelSpeedSetpoint = newSpeed;} 

  private void adjustSpeeds () {
    double currentSpeed = getWheelSpeed();
    if (currentSpeed < MIN_WHEEL_SPEED) {
        setWheelSpeed(MIN_WHEEL_SPEED);

    }

    double currentPivotSpeed = getPivotSpeed();
    if (currentPivotSpeed < MIN_PIVOT_SPEED) {
        setWheelSpeed(MIN_PIVOT_SPEED);

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
      
    SystemState newState = handleStateTransition();
    if (newState != systemState) {
         systemState = newState;
    }

    // Run outputs based on current system state
    switch (systemState) {
      case FEEDING:
          io.setIntakeSpeed(0.5);
          io.setPositionSetpoint(15);
          break;
      case REVERSING:
          io.setIntakeSpeed(-wheelSpeedSetpoint);
          io.setPivotSpeed(-pivotSpeedSetpoint);
          break;
      case IDLED:
      default:
          io.setIntakeSpeed(0.0);
          io.setPivotSpeed(0.0);
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

  // Public control methods
  public void feed(double wheelSpeed, double pivotSpeed) {
    this.wheelSpeedSetpoint = wheelSpeed;
    this.pivotSpeedSetpoint = pivotSpeed;
    setWantedState(WantedState.FEEDING);
  }

  public void reverse(double wheelSpeed, double pivotSpeed) {
    this.wheelSpeedSetpoint = wheelSpeed;
    this.pivotSpeedSetpoint = pivotSpeed;
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










/* public enum GroundIntakeSubsystem {

}
*/
