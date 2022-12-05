package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;


public class SwerveModule {
    public final SteeringMotor steeringMotor;
    public final SteeringSensor steeringSensor;
    public final DriveMotor driveMotor;
    public final Translation2d moduleXYTranslation;
    public final SwerveModuleSim simModule;
    public String steerMode = "INTEGRATED";
    public boolean hasSwerveSeedingOccurred=false;
    public boolean hasCANCoderBeenSetToAbs = false;
    public double swerveSeedingRetryCount = 0;
    public  StatorCurrentLimitConfiguration steerCurrentLimitConfigurationEnable;
    public  StatorCurrentLimitConfiguration steerCurrentLimitConfigurationDisable;
    public  StatorCurrentLimitConfiguration driveCurrentLimitConfigurationEnable;
    public  StatorCurrentLimitConfiguration driveCurrentLimitConfigurationDisable;
    public SwerveModuleState mCurrentModuleState;
    public static SimpleMotorFeedforward driveMotorFeedforward = new SimpleMotorFeedforward(SwerveConstants.kS, SwerveConstants.kV, SwerveConstants.kA);

    /**
     * @param driveMotor
     * @param steeringMotor
     * @param steeringSensor
     * @param moduleXYTranslation X is NorthSouth and Y is EastWest
     *     
     * Helpful hints:
     * 1. when determining your steering motor offsets first rotate 
     *    all modules to face a certain direction (inward/outward/left/right)
     * 2. Once that's done make sure you determine which drive motors need to go
     *    clockwise positive/negative
     * 3. NOW, ur ready to play with the offsets
     * 4. Use phoenix tuner to determin PID coefficients for EACH wheel, each wheel may
     *    may be slightly to vastly different
     * 
     */
    public SwerveModule (DriveMotor driveMotor, SteeringMotor steeringMotor, SteeringSensor steeringSensor, Translation2d moduleXYTranslation){
        this.steeringMotor = steeringMotor;
        this.steeringSensor = steeringSensor;
        this.driveMotor = driveMotor;
        this.moduleXYTranslation = moduleXYTranslation;

        if (RobotBase.isSimulation()) {
            simModule = new SwerveModuleSim(driveMotor, steeringMotor, steeringSensor);
        } else {
            simModule = null;
        }

        swerveModuleInit();
    }
    
    private void swerveModuleInit(){


        steerCurrentLimitConfigurationEnable  = new StatorCurrentLimitConfiguration();
        steerCurrentLimitConfigurationDisable = new StatorCurrentLimitConfiguration();
        driveCurrentLimitConfigurationEnable  = new StatorCurrentLimitConfiguration();
        driveCurrentLimitConfigurationDisable = new StatorCurrentLimitConfiguration();

        steerCurrentLimitConfigurationEnable.enable = true;
        steerCurrentLimitConfigurationEnable.triggerThresholdCurrent = 60;
        steerCurrentLimitConfigurationEnable.triggerThresholdTime = .1;
        steerCurrentLimitConfigurationEnable.currentLimit = 30;

        steerCurrentLimitConfigurationDisable.enable = false;
        driveCurrentLimitConfigurationDisable.enable = false; 

        driveCurrentLimitConfigurationEnable.enable = true;
        driveCurrentLimitConfigurationEnable.triggerThresholdCurrent = 80;
        driveCurrentLimitConfigurationEnable.triggerThresholdTime = .1;
        driveCurrentLimitConfigurationEnable.currentLimit = 60;
        
        //Setup the drive motor, but first set EVERYTHING to DEFAULT
        //Commented this out for boot speed savings
        //mDriveMotor.configFactoryDefault();

        driveMotor.setInverted(driveMotor.kWheelDirectionType);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, SwerveConstants.kDefaultTimeout);
        driveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationDisable, 1000);

        //Setup the Steering Sensor
        CANCoderConfiguration myCanCoderConfig = new CANCoderConfiguration();
        myCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        myCanCoderConfig.sensorDirection = false;
        myCanCoderConfig.magnetOffsetDegrees = steeringSensor.kOffsetDegrees;
        myCanCoderConfig.sensorCoefficient = 360.0/4096.0;
        myCanCoderConfig.unitString = "deg";
        myCanCoderConfig.sensorTimeBase= SensorTimeBase.PerSecond;
        myCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        
        
        if(steeringSensor.configAllSettings(myCanCoderConfig,1000)==ErrorCode.OK){
            System.out.println("CANCoder " + steeringSensor.getDeviceID() + " configured.");
        } else {
            System.out.println("WARNING! CANCoder " + steeringSensor.getDeviceID() + " NOT configured correctly! Error: " + steeringSensor.getLastError());
        }

        //First Attempt at seeding
        if(steeringSensor.setPositionToAbsolute(1000)==ErrorCode.OK){
            hasCANCoderBeenSetToAbs = true;
        }
        
        //Setup the the closed-loop PID for the steering module loop
        
        TalonFXConfiguration mySteeringMotorConfiguration = new TalonFXConfiguration();
        mySteeringMotorConfiguration.feedbackNotContinuous = false;
        mySteeringMotorConfiguration.slot0.kP = steeringMotor.kGAINS.kP;
        mySteeringMotorConfiguration.slot0.kI = steeringMotor.kGAINS.kI;
        mySteeringMotorConfiguration.slot0.kD = steeringMotor.kGAINS.kD;
        mySteeringMotorConfiguration.slot0.kF = steeringMotor.kGAINS.kF;
        mySteeringMotorConfiguration.slot0.allowableClosedloopError = SwerveConstants.kDefaultClosedLoopError;
        mySteeringMotorConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        mySteeringMotorConfiguration.remoteFilter0.remoteSensorDeviceID= steeringSensor.getDeviceID();

        if(steeringMotor.configAllSettings(mySteeringMotorConfiguration,1000)==ErrorCode.OK)   {
            System.out.println("Steer Motor " + steeringMotor.getDeviceID() + " configured.");
        } else {
            System.out.println("WARNING! Steer Motor  " + steeringMotor.getDeviceID() + " NOT configured correctly!");
        }

        steeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationDisable, 1000);
        steeringMotor.setInverted(TalonFXInvertType.Clockwise);
        steeringMotor.configSelectedFeedbackCoefficient(1.0/SwerveConstants.TICKSperTALONFX_STEERING_DEGREE,0,SwerveConstants.kDefaultTimeout);
        steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,SwerveConstants.kDefaultTimeout);
    }

    /**
     * Set all the status frames high in order relieve
     */
    public void setSwerveModuleCANStatusFrames(){
        
        if(driveMotor.hasResetOccurred()){
            int mycounter = 0;
            
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
            System.out.println("RESET DETECTED FOR TALONFX " + driveMotor.getDeviceID() + " Errors: " + mycounter);
        }
        if(steeringMotor.hasResetOccurred()){
            int mycounter = 0;
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100) !=ErrorCode.OK) {mycounter++;}
            if(steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100) !=ErrorCode.OK) {mycounter++;}
            System.out.println("RESET DETECTED FOR TALONFX " + steeringMotor.getDeviceID()+ " Errors: " + mycounter);
        }
    }

    public void enableCurrentLimiting(){
        driveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationEnable, 250);
        steeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationEnable, 250);
    }

    public void disableCurrentLimiting(){
        driveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationDisable, 250);
        steeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationDisable, 250);
    }

    public void setModuleToCoast(){
        driveMotor.setNeutralMode(NeutralMode.Coast);
        steeringMotor.setNeutralMode(NeutralMode.Coast);
    }
    public void setModuleToBrake(){
        driveMotor.setNeutralMode(NeutralMode.Brake);
        steeringMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * This method takes our best crack at seeding the angle from CANCoder to
     * Integrated Sensor on the Steering Motor (cuz faster/snappier).  
     */
    public void seedCANCoderAngleToMotorAngle() {

        // FIRST
        if(!hasSwerveSeedingOccurred && swerveSeedingRetryCount <=50) {
            if(!hasCANCoderBeenSetToAbs && steeringSensor.getAbsolutePosition() != steeringSensor.getPosition()){
                if(steeringSensor.setPositionToAbsolute(1000)==ErrorCode.OK) {
                    hasCANCoderBeenSetToAbs = true;
                } else {
                    swerveSeedingRetryCount++;
                    System.out.println("ERROR: COULDN'T SET THE CANCODER POSITION TO ABSOLUTE ANGLE! CANCODER: " + steeringSensor.getDeviceID() + " ERROR: " + steeringSensor.getLastError());
                }
            } else if(steeringMotor.setSelectedSensorPosition(steeringSensor.getPosition(),0,1000) == ErrorCode.OK){
                    System.out.println("Seeded Sensor values from " + steeringSensor.getDeviceID() + ": " + steeringSensor.getPosition() + " to " + steeringMotor.getDeviceID() + ": " + steeringMotor.getSelectedSensorPosition());
                    steeringSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255, 1000);
                    if(steeringSensor.getPosition()- steeringMotor.getSelectedSensorPosition() < 2.0){
                        hasSwerveSeedingOccurred=true;
                    }
            } else {
                System.out.println("ERROR: COULDNT SEED VALUES FOR STEER MOTOR: " + steeringMotor.getDeviceID() + " RETRY COUNT: " + swerveSeedingRetryCount);
                swerveSeedingRetryCount++;
            }

        }  else if (swerveSeedingRetryCount >50 && !steerMode.equals("REMOTE")) {
            System.out.println("ERROR: COULDNT SET POSITION TO ABSOLUTE! CANCODER: " + steeringSensor.getDeviceID());
            switchToCANCoderSteer();
        }

    }

    public String getSteeringSelectedSensor(){
        return steerMode;
    }

    public void switchToCANCoderSteer(){
        steeringSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 1000);
        steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,0,255);
        steerMode = "REMOTE";
    }

    public void switchToIntegratedSteer(){
        steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,SwerveConstants.kDefaultTimeout); 
        hasSwerveSeedingOccurred = false;  
        swerveSeedingRetryCount = 0;  

        steerMode = "INTEGRATED";
        seedCANCoderAngleToMotorAngle();

    }
    
    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState kState = desiredState;
        if(Preferences.getBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING)){
            kState = optimize(desiredState, new Rotation2d(Math.toRadians(steeringMotor.getSelectedSensorPosition())));
        }
        
        double convertedspeed = kState.speedMetersPerSecond*(SwerveConstants.SECONDSper100MS)*SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION/(SwerveConstants.METERSperWHEEL_REVOLUTION);           
        setSteeringAngle(kState.angle.getDegrees());
        
        if (SwerveConstants.BOT_IS_NOT_CHARACTERIZED){

            driveMotor.set(ControlMode.PercentOutput, kState.speedMetersPerSecond/SwerveConstants.MAX_SPEED_METERSperSECOND); 
            
        } else {
            //System.out.println(driveMotorFeedforward.calculate(kState.speedMetersPerSecond));
            driveMotor.setVoltage(driveMotorFeedforward.calculate(kState.speedMetersPerSecond)) ;              
        }
        
    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity()/SwerveConstants.SECONDSper100MS/SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION*SwerveConstants.METERSperWHEEL_REVOLUTION, Rotation2d.fromDegrees(steeringMotor.getSelectedSensorPosition()));
    }

      /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition()/SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION*SwerveConstants.METERSperWHEEL_REVOLUTION, new Rotation2d(Math.toRadians(steeringMotor.getSelectedSensorPosition())));
    }
    
    /** 
     * This method takes in setAngle in DEGREES, 
     * 
     * compares that angle with the current position of 
     * the swerve module and decides which direction to 
     * rotate the module.
     * 
     * The angle is then converted to sensor units (4096 
     * equals 1 full rotation) units equals and fed
     * to the steering motor to update.
     * @param _angle (IN DEGREES)
     */
    public void setSteeringAngle(double _angle){
        //double newAngleDemand = _angle;
        double currentSensorPosition = steeringMotor.getSelectedSensorPosition();
        double remainder = Math.IEEEremainder(currentSensorPosition, 360);
        double newAngleDemand = _angle + currentSensorPosition -remainder;
        
        //System.out.println(mSteeringMotor.getSelectedSensorPosition()-remainder );
        if(newAngleDemand - currentSensorPosition > 180.1){
              newAngleDemand -= 360;
          } else if (newAngleDemand - currentSensorPosition < -180.1){
              newAngleDemand += 360;
          }
        steeringMotor.set(ControlMode.Position, newAngleDemand );
    }
  
    public SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90.0) {  //SJV: If this doesn'twork try 360
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
      } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    
    public static class SteeringMotor extends WPI_TalonFX{  
        public SwerveConstants.Gains kGAINS;

        public SteeringMotor(int _talonID, SwerveConstants.Gains _gains) {
            super(_talonID, "Swerve");
            kGAINS = _gains;
        }
    }

    public static class DriveMotor extends WPI_TalonFX{
        public TalonFXInvertType kWheelDirectionType;
        public SwerveConstants.Gains kGAINS;
        public DriveMotor (int _talonID, TalonFXInvertType _direction, SwerveConstants.Gains _gains){
            super(_talonID);
            kWheelDirectionType = _direction;
            kGAINS=_gains;
        }
    }

    public static class SteeringSensor extends CANCoder{
        public double kOffsetDegrees;

        public SteeringSensor (int _sensorID, double _offsetDegrees){
            super(_sensorID, "Swerve");
            kOffsetDegrees = _offsetDegrees;
         }       
    }


  
}
