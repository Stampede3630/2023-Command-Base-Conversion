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
      new Translation2d(SwerveConstants.WHEEL_BASE_METERS/2, SwerveConstants.TRACK_WIDE/2));
   
    public SwerveModule FrontRightSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.FRDriveID, SwerveConstants.FRInvertType, SwerveConstants.FRDriveGains), 
      new SwerveModule.SteeringMotor(SwerveConstants.FRSteerID, SwerveConstants.FRSteerGains), 
      new SwerveModule.SteeringSensor(SwerveConstants.FRSensorID,SwerveConstants.FRSensorOffset),
      new Translation2d(SwerveConstants.WHEEL_BASE_METERS/2, -SwerveConstants.TRACK_WIDE/2) );
  
    public SwerveModule BackLeftSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.BLDriveID,SwerveConstants.BLInvertType, SwerveConstants.BLDriveGains), 
      new SwerveModule.SteeringMotor(SwerveConstants.BLSteerID, SwerveConstants.BLSteerGains), 
      new SwerveModule.SteeringSensor(SwerveConstants.BLSensorID,SwerveConstants.BLSensorOffset),
      new Translation2d(-SwerveConstants.WHEEL_BASE_METERS/2, SwerveConstants.TRACK_WIDE/2));
    
    public SwerveModule BackRightSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.BRDriveID,SwerveConstants.BRInvertType, SwerveConstants.BRDriveGains) , 
      new SwerveModule.SteeringMotor(SwerveConstants.BRSteerID, SwerveConstants.BRSteerGains), 
      new SwerveModule.SteeringSensor(SwerveConstants.BRSensorID,SwerveConstants.BRSensorOffset),
      new Translation2d(-SwerveConstants.WHEEL_BASE_METERS/2, -SwerveConstants.TRACK_WIDE/2));



    public SwerveDriveKinematics m_kinematics = 
        new SwerveDriveKinematics(
            FrontLeftSwerveModule.moduleXYTranslation, 
            FrontRightSwerveModule.moduleXYTranslation, 
            BackLeftSwerveModule.moduleXYTranslation, 
            BackRightSwerveModule.moduleXYTranslation);

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

    public void checkAndSeedALLSwerveAngles() {
      FrontLeftSwerveModule.seedCANCoderAngleToMotorAngle();
      BackLeftSwerveModule.seedCANCoderAngleToMotorAngle();
      FrontRightSwerveModule.seedCANCoderAngleToMotorAngle();
      BackRightSwerveModule.seedCANCoderAngleToMotorAngle();      
    }

    public void switchToRemoteSteering() {
      FrontLeftSwerveModule.switchToCANCoderSteer();
      BackLeftSwerveModule.switchToCANCoderSteer();
      FrontRightSwerveModule.switchToCANCoderSteer();
      BackRightSwerveModule.switchToCANCoderSteer();      
    }

    public void switchToIntegratedSteer(){
      FrontLeftSwerveModule.switchToIntegratedSteer();
      BackLeftSwerveModule.switchToIntegratedSteer();
      FrontRightSwerveModule.switchToIntegratedSteer();
      BackRightSwerveModule.switchToIntegratedSteer();    
    }

    public void activateDefensiveStop() {
      FrontLeftSwerveModule.setSteeringAngle(45);
      FrontLeftSwerveModule.driveMotor.set(ControlMode.PercentOutput, 0);

      FrontRightSwerveModule.setSteeringAngle(135);
      FrontRightSwerveModule.driveMotor.set(ControlMode.PercentOutput, 0);

      BackLeftSwerveModule.setSteeringAngle(135);
      BackLeftSwerveModule.driveMotor.set(ControlMode.PercentOutput, 0);

      BackRightSwerveModule.setSteeringAngle(45);
      BackRightSwerveModule.driveMotor.set(ControlMode.PercentOutput, 0);
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
      Math.abs(FrontLeftSwerveModule.driveMotor.getSelectedSensorVelocity())  < 100 &&
      Math.abs(BackLeftSwerveModule.driveMotor.getSelectedSensorVelocity())   < 100 &&
      Math.abs(FrontRightSwerveModule.driveMotor.getSelectedSensorVelocity()) < 100 &&
      Math.abs(BackRightSwerveModule.driveMotor.getSelectedSensorVelocity())  < 100) {
        FrontRightSwerveModule.setModuleToCoast();
        BackRightSwerveModule.setModuleToCoast();
        FrontLeftSwerveModule.setModuleToCoast();
        BackLeftSwerveModule.setModuleToCoast();
        NeutralMode = "Coast";
      }
  }

  public void setToBrake(){
    FrontRightSwerveModule.setModuleToBrake();
    BackRightSwerveModule.setModuleToBrake();
    FrontLeftSwerveModule.setModuleToBrake();
    BackLeftSwerveModule.setModuleToBrake();
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

    
    

