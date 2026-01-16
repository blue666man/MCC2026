package frc.robot.subsystems.GroundIntake;






//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix6.hardware.*;


public class GroundIntakeIOTalonFX implements GroundIntakeIO{
    private final TalonFX pivotMotor;
    private final TalonFX intakeMotor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward ff = new ArmFeedforward(0., 0.035, 0);
    PIDController controller= new PIDController(2.4067, 0, 0.0);
   

    public GroundIntakeIOTalonFX(int pivotID, int intakeID) {
        this.pivotMotor = new TalonFX(pivotID);
        this.intakeMotor = new TalonFX(intakeID);
        encoder = new DutyCycleEncoder(0);
      }
    
    
    @Override
    public void setIntakeSpeed(double speed) {
      this.intakeMotor.set(speed);
    }
    @Override
    public void setPivotSpeed(double speed) {
      this.pivotMotor.set(speed);
     }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
       //SPARK MAX VERSION: inputs.intakeSpeed = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
       inputs.intakeSpeed = intakeMotor.getMotorVoltage().getValueAsDouble();
       //SPARK MAX VERSION: inputs.encoderPosition = pivotMotor.getOutputCurrent();
       inputs.encoderPosition = pivotMotor.getStatorCurrent().getValueAsDouble();
    }
    @Override
    public void refreshData() {
        // Not required for Spark MAX, but useful for manual telemetry push or debug logging
        //SPARK MAX VERSION: SmartDashboard.putNumber("GroundIntake/WheelSpeed", intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        //SPARK MAX VERSION: SmartDashboard.putNumber("GroundIntake/EncoderPosition", pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
        SmartDashboard.putNumber("GroundIntake/WheelSpeed", intakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/EncoderPosition", pivotMotor.getMotorVoltage().getValueAsDouble());
    }
    /* @Override
    public double getEncoderVal() {
      return this.pivotMotor.getEncoder().getPosition();
    } */

    @Override
    public double getEncoderVal() {
    // getPosition() returns a StatusSignal; getValueAsDouble() converts it to a numeric value.
    return this.pivotMotor.getPosition().getValueAsDouble();
}

    @Override
    public void setPositionSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);

        double currentPosition = getEncoderVal(); // same as in updateInputs()
        double output = controller.calculate(currentPosition);

        pivotMotor.set(output); // same direction for both
    }


}



