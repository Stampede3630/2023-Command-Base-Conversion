// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
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
  QuadFalconSwerveDrive m_swerveDrive = new QuadFalconSwerveDrive();
  AHRS gyro;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    m_swerveDrive.driveRobotInit();
    m_swerveDrive.checkAndSetSwerveCANStatus();
    m_swerveDrive.checkAndZeroSwerveAngle();
    gyro = new AHRS(Port.kMXP);
  }

  @Override
  public void periodic() {
    m_swerveDrive.checkAndSetSwerveCANStatus();
  }

  /**
   * @param xySpeedsMetersPerSec (X is Positive forward, Y is Positive Right)
   * @param rRadiansPerSecond 
   * @param fieldRelative (SUGGESTION: Telop use field centric, AUTO use robot centric)
   */
  public void setDriveSpeeds(Translation2d xySpeedsMetersPerSec, double rRadiansPerSecond, boolean fieldRelative){
    SwerveModuleState[] swerveModuleStates =
      m_swerveDrive.m_kinematics.toSwerveModuleStates(
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
    m_swerveDrive.setModuleSpeeds(swerveModuleStates);
  }


  
  
  /**
   * @return a Rotation2d populated by the gyro readings or estimated by encoder wheels (if gyro is disconnected)
   */
  public Rotation2d getRobotAngle(){
    if (gyro.isConnected()){
        return gyro.getRotation2d();
    } else {
        try {
        //System.out.println( deltaTime);
        return m_poseEstimator.getEstimatedPosition().getRotation().rotateBy(new Rotation2d(m_swerveDrive.m_kinematics.toChassisSpeeds(m_swerveDrive.getModuleStates()).omegaRadiansPerSecond *Robot.deltaTime));
        } catch (Exception e) {
        return new Rotation2d();        
        }
    }
    gyro.
  }

  public void updateOdometry(){
    m_poseEstimator.update(getRobotAngle(), 
    quadFalconSwerveDrive.getModuleStates(), 
    quadFalconSwerveDrive.getModulePositions());
  }

}
