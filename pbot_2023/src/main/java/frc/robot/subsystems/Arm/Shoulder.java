package frc.robot.subsystems.Arm;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ElbowConstants;

public class Shoulder {
    public CANSparkMax LeftShoulderMotor;
    public CANSparkMax RightShoulderMotor;
    public AbsoluteEncoder ShoulderEncoder;
    public ProfiledPIDController ShoulderController;
    private ArmFeedforward shoulderFF;
    public Shoulder(){
        LeftShoulderMotor.follow(RightShoulderMotor, false);
        LeftShoulderMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightShoulderMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightShoulderMotor.setInverted(false);
        RightShoulderMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
        LeftShoulderMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
        ShoulderEncoder.setPositionConversionFactor(ElbowConstants.kElbowPositionConversionFactor);
        
        ShoulderEncoder = RightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
        RightShoulderMotor.setIdleMode(IdleMode.kBrake);
        LeftShoulderMotor.setIdleMode(IdleMode.kBrake);
        ShoulderController = new ProfiledPIDController(0.001, 0,  0, new TrapezoidProfile.Constraints(0.5, 0.5));
        shoulderFF = new ArmFeedforward(0,0.47,4.68,0.04);
        ShoulderEncoder.setZeroOffset(ElbowConstants.kElbowEncoderZeroOffset);
    }
    public double convertTicksToAngle(double angle){
        double newAngle = angle;
        newAngle -= ElbowConstants.kElbowEncoderZeroOffset;
        return newAngle / ElbowConstants.kElbowGearRatio;
    }
    public double getAngle(){
        return convertTicksToAngle(ShoulderEncoder.getPosition());
    }
    public void setPos(double goal){
        ShoulderController.setGoal(goal);
    }
    public void setAngle(){
        RightShoulderMotor.setVoltage(ShoulderController.calculate(getAngle(),
        ShoulderController.getGoal()));
        shoulderFF.calculate(ShoulderController.getSetpoint().position, 0);
    }
}


