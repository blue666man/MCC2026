// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackFuel extends Command {
  /** Creates a new TrackFuel. */
  CommandSwerveDrivetrain drive;
  Limelight intakeLL;
  PIDController txController = new PIDController(0.1, 0, 0);
  PIDController tyController = new PIDController(0.1, 0, 0);
  public TrackFuel(CommandSwerveDrivetrain drive, Limelight intakeLL) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.intakeLL = intakeLL;
    SmartDashboard.putData("PIDX", txController);
    SmartDashboard.putData("PIDY", tyController);
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    txController.setSetpoint(2);
    txController.setTolerance(0.3);
    tyController.setSetpoint(-20);
    tyController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setControl(new SwerveRequest.RobotCentric()
    .withVelocityX(-tyController.calculate(intakeLL.getTY()))
    .withVelocityY(txController.calculate(intakeLL.getTX()))
    .withRotationalRate(0)
    );
    System.out.println("Calculated: " + txController.calculate(intakeLL.getTX()) + ", " + tyController.calculate(intakeLL.getTY()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
