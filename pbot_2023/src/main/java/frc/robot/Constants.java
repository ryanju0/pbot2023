// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class ArmConstants{

    public static final class ElbowConstants{
      public static final int kRightElbowMotorCanId = 11;
      public static final int kLeftElbowMotorCanId = 12; 
      public static final int kElbowMotorCurrentLimit = 15; 
      public static final double kElbowGearRatio = 225;
      public static final double kElbowPositionConversionFactor = (2*Math.PI) * kElbowGearRatio;
      public static final double kElbowEncoderZeroOffset = 41.4360188;
      public static final boolean kShoulderMotorInverted = true; //base joint encoder inverted
      public static final boolean kShoulderEncoderInverted = true;
      //Controller Constants
      public static final double kElbowMaxVelocity = 5500;
      public static final double kElbowMaxAcceleration = 4000;
      public static final double kElbowTolerance = 6;
      public static final double kElbowFF = 0.00008;
      public static final double kElbowP = 0.00006;
      public static final double kElbowI = 0.0;
      public static final double kElbowD = 0.00012;
    }
    public static final class ShoulderConstants{
      public static final int kRightShoulderMotorCanId = 10;
      public static final int kLeftShoulderMotorCanId = 9;
      public static final int kShoulderMotorCurrentLimit = 40;
      public static final double kShoulderGearRatio = 240;
      public static final double kShoulderPositionConversionFactor = (2*Math.PI) * kShoulderGearRatio;
      public static final double kShoulderEncoderZeroOffset = 313.4707425;
      public static final boolean kShoulderMotorInverted = true; //base joint encoder inverted
      public static final boolean kShoulderEncoderInverted = true;

          //Controller Constants
      public static final double kShoulderMaxVelocity = 5000;
      public static final double kShoulderMaxAcceleration = 4000;
      public static final double kShoulderTolerance = 6;
      public static final double kShoulderFF = 0.00007;
      public static final double kShoulderP = 0.00000;
      public static final double kShoulderI = 0;
      public static final double kShoulderD = 0.0000;

      
    } 

  }
}
