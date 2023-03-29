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

public class Elbow {
    public CANSparkMax LeftElbowMotor;
    public CANSparkMax RightElbowMotor;
    public AbsoluteEncoder ElbowEncoder;
    public ProfiledPIDController ElbowController;
    private ArmFeedforward elbowFF;
    public Elbow(){
        LeftElbowMotor.follow(RightElbowMotor, false);
        LeftElbowMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightElbowMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightElbowMotor.setInverted(false);
        RightElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
        LeftElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
        ElbowEncoder.setPositionConversionFactor(ElbowConstants.kElbowPositionConversionFactor);
        
        ElbowEncoder = RightElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        RightElbowMotor.setIdleMode(IdleMode.kBrake);
        LeftElbowMotor.setIdleMode(IdleMode.kBrake);
        ElbowController = new ProfiledPIDController(0.001, 0,  0, new TrapezoidProfile.Constraints(0.1, 0.1));
        elbowFF = new ArmFeedforward(0,0.35,4.38,0.03);
        ElbowEncoder.setZeroOffset(ElbowConstants.kElbowEncoderZeroOffset);
    }
    public double convertTicksToAngle(double angle){
        double newAngle = angle;
        newAngle -= ElbowConstants.kElbowEncoderZeroOffset;
        return newAngle / ElbowConstants.kElbowGearRatio;
    }
    public double getAngle(){
        return convertTicksToAngle(ElbowEncoder.getPosition());
    }
    public void setPos(double goal){
        ElbowController.setGoal(goal);
    }
    public void setAngle(){
        RightElbowMotor.setVoltage(ElbowController.calculate(getAngle(),
        ElbowController.getGoal()));
        elbowFF.calculate(ElbowController.getSetpoint().position, 0);
    }
}

