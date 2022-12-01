package frc.robot.subsystems.swerve;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.util.SimGyroSensorModel;




public class QuadFalconSwerveDrive {
    public String NeutralMode = "Brake";
    public TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    public SwerveModule FrontLeftSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.FLDriveID,SwerveConstants.FLInvertType, SwerveConstants.FLDriveGains),
      new SwerveModule.SteeringMotor(SwerveConstants.FLSteerID, SwerveConstants.FLSteerGains), 
      new SwerveModule.SteeringSensor(SwerveConstants.FLSensorID,SwerveConstants.FLSensorOffset),
      new Translation2d(SwerveConstants.WHEEL_BASE_METERS/2, -SwerveConstants.TRACK_WIDE/2));
   
    public SwerveModule FrontRightSwerveModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.FRDriveID, SwerveConstants.FRInvertType, SwerveConstants.FRDriveGains), 
        new SwerveModule.SteeringMotor(SwerveConstants.FRSteerID, SwerveConstants.FRSteerGains), 
        new SwerveModule.SteeringSensor(SwerveConstants.FRSensorID,SwerveConstants.FRSensorOffset),
        new Translation2d(SwerveConstants.WHEEL_BASE_METERS/2, SwerveConstants.TRACK_WIDE/2) );

    public SwerveModule BackRightSwerveModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.BRDriveID,SwerveConstants.BRInvertType, SwerveConstants.BRDriveGains) , 
        new SwerveModule.SteeringMotor(SwerveConstants.BRSteerID, SwerveConstants.BRSteerGains), 
        new SwerveModule.SteeringSensor(SwerveConstants.BRSensorID,SwerveConstants.BRSensorOffset),
        new Translation2d(-SwerveConstants.WHEEL_BASE_METERS/2, SwerveConstants.TRACK_WIDE/2));

    public SwerveModule BackLeftSwerveModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.BLDriveID,SwerveConstants.BLInvertType, SwerveConstants.BLDriveGains), 
        new SwerveModule.SteeringMotor(SwerveConstants.BLSteerID, SwerveConstants.BLSteerGains), 
        new SwerveModule.SteeringSensor(SwerveConstants.BLSensorID,SwerveConstants.BLSensorOffset),
        new Translation2d(-SwerveConstants.WHEEL_BASE_METERS/2, -SwerveConstants.TRACK_WIDE/2));

    public SwerveDriveKinematics m_kinematics = 
        new SwerveDriveKinematics(
            FrontLeftSwerveModule.mTranslation2d, 
            FrontRightSwerveModule.mTranslation2d, 
            BackLeftSwerveModule.mTranslation2d, 
            BackRightSwerveModule.mTranslation2d);

    public final List<SwerveModule> SwerveModuleList = 
        List.of(
            FrontLeftSwerveModule, 
            FrontRightSwerveModule, 
            BackLeftSwerveModule, 
            BackRightSwerveModule);

    public void checkAndSetSwerveCANStatus(){
        FrontLeftSwerveModule.setSwerveModuleCANStatusFrames();
        FrontRightSwerveModule.setSwerveModuleCANStatusFrames();
        BackLeftSwerveModule.setSwerveModuleCANStatusFrames();
        BackRightSwerveModule.setSwerveModuleCANStatusFrames();
    }

    public void checkAndZeroSwerveAngle() {
      FrontLeftSwerveModule.zeroSwerveAngle();
      BackLeftSwerveModule.zeroSwerveAngle();
      FrontRightSwerveModule.zeroSwerveAngle();
      BackRightSwerveModule.zeroSwerveAngle();      
    }

    public void activateDefensiveStop() {
      FrontLeftSwerveModule.setSteeringAngle(45);
      FrontLeftSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

      FrontRightSwerveModule.setSteeringAngle(135);
      FrontRightSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

      BackLeftSwerveModule.setSteeringAngle(135);
      BackLeftSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

      BackRightSwerveModule.setSteeringAngle(45);
      BackRightSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setModuleSpeeds (SwerveModuleState[] _swerveModuleStates) {
        FrontLeftSwerveModule.setDesiredState(_swerveModuleStates[0]);
        FrontRightSwerveModule.setDesiredState(_swerveModuleStates[1]);
        BackLeftSwerveModule.setDesiredState(_swerveModuleStates[2]);
        BackRightSwerveModule.setDesiredState(_swerveModuleStates[3]);
    }

  /**
   * MUST BE ADDED TO PERIODIC (NOT INIT METHODS)
   * sets all the talons (steer and drive motors) to coast.
   * This allows for easy moving of the robot
   */
  public void setToCoast(){ 
    if (NeutralMode == "Brake" &&
      Math.abs(FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity())  < 100 &&
      Math.abs(BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity())   < 100 &&
      Math.abs(FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()) < 100 &&
      Math.abs(BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity())  < 100) {
        FrontRightSwerveModule.swerveDisabledInit();
        BackRightSwerveModule.swerveDisabledInit();
        FrontLeftSwerveModule.swerveDisabledInit();
        BackLeftSwerveModule.swerveDisabledInit();
        NeutralMode = "Coast";
      }
  }

  public void setToBrake(){
    FrontRightSwerveModule.swerveEnabledInit();
    BackRightSwerveModule.swerveEnabledInit();
    FrontLeftSwerveModule.swerveEnabledInit();
    BackLeftSwerveModule.swerveEnabledInit();
    NeutralMode = "Brake";
  }

  public void enableCurrentLimiting(){
    FrontRightSwerveModule.enableCurrentLimiting();
    BackRightSwerveModule.enableCurrentLimiting();
    FrontLeftSwerveModule.enableCurrentLimiting();
    BackLeftSwerveModule.enableCurrentLimiting();
  }

  public void disableCurrentLimiting(){
    FrontRightSwerveModule.disableCurrentLimiting();
    BackRightSwerveModule.disableCurrentLimiting();
    FrontLeftSwerveModule.disableCurrentLimiting();
    BackLeftSwerveModule.disableCurrentLimiting();
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
        FrontLeftSwerveModule.getSwerveModuleState(),
        FrontRightSwerveModule.getSwerveModuleState(),
        BackLeftSwerveModule.getSwerveModuleState(),
        BackRightSwerveModule.getSwerveModuleState()
    };
  }
  
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
        FrontLeftSwerveModule.getPosition(),
        FrontRightSwerveModule.getPosition(),
        BackLeftSwerveModule.getPosition(),
        BackRightSwerveModule.getPosition()
    };
  }

}

    
    

