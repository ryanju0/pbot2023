package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LimelightHelpers;


public class Limelight extends SubsystemBase{
    String limelightName = "limelight"; 
    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = networkTable.getEntry("tx");
    NetworkTableEntry ty = networkTable.getEntry("ty");
    NetworkTableEntry ta = networkTable.getEntry("ta");
    private double GoalHeight = 0;

    public Limelight(){
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
	public double getXOffset() {
    return LimelightHelpers.getTX(limelightName);
	}
  public double getYOffset() {
		return LimelightHelpers.getTY(limelightName);
	}
  public double getGoalHeight() {
		return GoalHeight;
	}
  public void setGoalHeight(double GoalHeight) {
		this.GoalHeight = GoalHeight;
	}
  public void alignPipeline() {
		setGoalHeight(LimelightConstants.kMiddleRetroTapeHeight);
		LimelightHelpers.setPipelineIndex(limelightName, 0);
	}

  @Override
  public void periodic(){
    
  }
}
