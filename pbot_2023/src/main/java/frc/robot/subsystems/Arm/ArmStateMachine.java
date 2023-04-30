
package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmStateMachine extends SubsystemBase{
    private Arm m_arm;
    public enum ArmState {
        BACK, FRONT
      }
    private ArmState armState = ArmState.BACK;
    public ArmState targetArmState = ArmState.BACK;
    public ArmStateMachine(Arm m_arm) {
        this.m_arm = m_arm;
    }
    public void setTargetArmState(ArmState targetArmState) {
        if(this.targetArmState != targetArmState){
          armState = this.targetArmState;
          this.targetArmState = targetArmState;
    
          callArmCommand();
        }
      }

      private void callArmCommand() {
        // Get arm command, retrieves the current action
        // .withInterruptBehavior what to do if the command gets interupted, in this case, stop running the command 
        // .sechedule() adds to list of commands to execute. 
        getArmCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
      }
      public Command setTargetArmStateCommand(ArmState targetArmState) {
        return new InstantCommand(() -> setTargetArmState(targetArmState));
      }
    public Command getArmCommand(){
    switch (armState){
        case BACK:
            return new InstantCommand(() -> m_arm.setArmPos(Units.degreesToRadians(82),Units.degreesToRadians(135),4));
        case FRONT:
            return new InstantCommand(() -> m_arm.setArmPos(Units.degreesToRadians(54),Units.degreesToRadians(-14),4));
            default:
            return new InstantCommand();
    }
    }
}
