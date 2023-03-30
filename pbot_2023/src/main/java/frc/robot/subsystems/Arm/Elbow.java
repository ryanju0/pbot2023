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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ElbowConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class Elbow extends SubsystemBase{
    public CANSparkMax LeftElbowMotor;
    public CANSparkMax RightElbowMotor;
    public AbsoluteEncoder ElbowEncoder;
    public ProfiledPIDController ElbowController;
    private ArmFeedforward elbowFF;
    private Constraints FarConstraints = new Constraints(12, 9);
    private Constraints CloseConstraints = new Constraints(36, 36);
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
        if(ElbowController.getP() == 0) {ElbowController.setP(5);}	
        ElbowController.setGoal(goal);	
    }
    public boolean atGoal() {	
        return ElbowController.atGoal();	
    }
    public void setTargetAngle(double targetAngle){
        Constraints selectedConstraint = (Math.abs(targetAngle - getAngle()) > Units.degreesToRadians(10) ? FarConstraints : CloseConstraints);
        ElbowController.setConstraints(selectedConstraint);

        ElbowController.setGoal(new State(targetAngle, 0));
    }
    public void setCalculatedVoltage(){	
        RightElbowMotor.setVoltage(ElbowController.calculate(getAngle(),ElbowController.getGoal()) + elbowFF.calculate(ElbowController.getSetpoint().position, 0));	
    }	
    @Override	
    public void periodic() {	
    setCalculatedVoltage();	
    }
}

