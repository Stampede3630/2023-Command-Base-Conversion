// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.QuadFalconSwerveDrive;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
  QuadFalconSwerveDrive swerveDrive = new QuadFalconSwerveDrive();
  public AHRS gyro;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    swerveDrive.driveRobotInit();
    swerveDrive.checkAndSetSwerveCANStatus();
    swerveDrive.checkAndZeroSwerveAngle();
    gyro = new AHRS(Port.kMXP);
  }

  @Override
  public void periodic() {
    swerveDrive.checkAndSetSwerveCANStatus();
  }

  public void setDriveSpeeds(Translation2d xySpeedsMetersPerSec, double rRadiansPerSecond, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates =
      swerveDrive.m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            xySpeedsMetersPerSec.getX(), 
                            xySpeedsMetersPerSec.getY(), 
                            rRadiansPerSecond, 
                            gyro.getRotation2d()
                        )
                        : new ChassisSpeeds(
                            xySpeedsMetersPerSec.getX(), 
                            xySpeedsMetersPerSec.getY(), 
                            rRadiansPerSecond)
                        );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERSperSECOND);
    swerveDrive.setModuleSpeeds(swerveModuleStates);
  }
  
  

}
