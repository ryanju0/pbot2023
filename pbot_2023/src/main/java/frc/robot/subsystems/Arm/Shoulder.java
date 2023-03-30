package frc.robot.subsystems.Arm;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;

public class Shoulder extends SubsystemBase{
    public CANSparkMax LeftShoulderMotor;
    public CANSparkMax RightShoulderMotor;
    public AbsoluteEncoder ShoulderEncoder;
    public ProfiledPIDController ShoulderController;
    private ArmFeedforward shoulderFF;
    private Constraints FarConstraints = new Constraints(10, 8);
    private Constraints CloseConstraints = new Constraints(18, 18);
    public Shoulder(){
        LeftShoulderMotor.follow(RightShoulderMotor, false);
        LeftShoulderMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightShoulderMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightShoulderMotor.setInverted(false);
        RightShoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderMotorCurrentLimit);
        LeftShoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderMotorCurrentLimit);
        ShoulderEncoder.setPositionConversionFactor(ShoulderConstants.kShoulderPositionConversionFactor);
        
        ShoulderEncoder = RightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
        RightShoulderMotor.setIdleMode(IdleMode.kBrake);
        LeftShoulderMotor.setIdleMode(IdleMode.kBrake);
        ShoulderController = new ProfiledPIDController(0.001, 0,  0, new TrapezoidProfile.Constraints(0.5, 0.5));
        shoulderFF = new ArmFeedforward(0,0.47,4.68,0.04);
        ShoulderEncoder.setZeroOffset(ShoulderConstants.kShoulderEncoderZeroOffset);
    }
    public void setPos(double goal){
        if(ShoulderController.getP() == 0) {ShoulderController.setP(5);}
        ShoulderController.setGoal(goal);
    }
    public double convertTicksToAngle(double angle){
        double newAngle = angle;
        newAngle -= ShoulderConstants.kShoulderEncoderZeroOffset;
        return newAngle / ShoulderConstants.kShoulderGearRatio;
    }
    public boolean atGoal() {
        return ShoulderController.atGoal();
    }
    public double getAngle(){
        return convertTicksToAngle(ShoulderEncoder.getPosition());
    }
    public void setTargetAngle(double targetAngle){
        Constraints selectedConstraint = (Math.abs(targetAngle - getAngle()) > Units.degreesToRadians(10) ? FarConstraints : CloseConstraints);
        ShoulderController.setConstraints(selectedConstraint);

        ShoulderController.setGoal(new State(targetAngle, 0));
    }
    public void setCalculatedVoltage(){
        RightShoulderMotor.setVoltage(ShoulderController.calculate(getAngle(),ShoulderController.getGoal()) + shoulderFF.calculate(ShoulderController.getSetpoint().position, 0));
    }
    @Override
    public void periodic() {
    setCalculatedVoltage();
    }
}


