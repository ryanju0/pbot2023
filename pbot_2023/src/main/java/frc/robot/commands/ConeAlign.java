package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class ConeAlign extends CommandBase{
    private DriveSubsystem m_drive;
    private Limelight m_limelight;

    private ProfiledPIDController yController;
    private int kPYAlign = 0;


    public ConeAlign(DriveSubsystem m_drive, Limelight m_limelight){
        this.m_drive = m_drive;
        this.m_limelight = m_limelight;

        yController = new ProfiledPIDController(kPYAlign, 0, 0, DriveConstants.kAutoControllerConstraints);
        yController.setGoal(0);
        yController.setTolerance(0,0);
    }
}