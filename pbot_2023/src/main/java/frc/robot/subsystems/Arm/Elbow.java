package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ElbowConstants;
public class Elbow {
    public CANSparkMax LeftElbowMotor;
    public CANSparkMax RightElbowMotor;
    public AbsoluteEncoder TopJointEncoder;
    public Elbow(){
        LeftElbowMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        RightElbowMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        LeftElbowMotor.follow(RightElbowMotor, false);
        RightElbowMotor.setInverted(false);
        RightElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
        LeftElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
    }
}
