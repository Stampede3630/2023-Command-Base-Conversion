// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveConstants;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  /*Controller setup.  For simulations google: x360CE */
  private final XboxController xBox = new XboxController(0);
  private final int xBoxXAxis = XboxController.Axis.kLeftY.value;
  private final int xBoxYAxis = XboxController.Axis.kLeftX.value;
  private final int xBoxRot = XboxController.Axis.kRightX.value;
  private boolean isIntegratedSteering = true;
  PIDController rotationController; 
  
  SwerveAutoBuilder autoBuilder;
  ArrayList<PathPlannerTrajectory> pathGroup;


     

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.


// This is just an example event map. It would be better to have a constant, global event map
  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Preferences.setBoolean("pFieldRelative", Constants.fieldRelative);
    Preferences.setBoolean("pAccelInputs", Constants.acceleratedInputs);
    Preferences.setDouble("pDriveGovernor", Constants.driveGovernor);
    Preferences.setBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING);
    Preferences.setDouble("pKPRotationController", SwerveConstants.kpRotationController);

    rotationController = new PIDController(Preferences.getDouble("pKPRotationController", SwerveConstants.kpRotationController),0,0);

    rotationController.enableContinuousInput(-180.0, 180.0);
    rotationController.setTolerance(2.0);


    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("1stBallPickup", new WaitCommand(2));
    eventMap.put("2ndBallPickup", new WaitCommand(2));
    eventMap.put("3rdBallPickup", new WaitCommand(2));

    s_SwerveDrive.setDefaultCommand( 
      s_SwerveDrive.joystickDriveCommand(
          () -> -xBox.getRawAxis(xBoxXAxis),
          () -> -xBox.getRawAxis(xBoxYAxis),
          () -> rotationInputController(),
          () -> Preferences.getDouble("pDriveGovernor", Constants.driveGovernor),
          () -> Preferences.getBoolean("pFieldRelative", Constants.fieldRelative),
          () -> Preferences.getBoolean("pAccelInputs", Constants.acceleratedInputs)
        )
    );
    autoBuilder = new SwerveAutoBuilder(
      s_SwerveDrive::getOdometryPose, // Pose2d supplier
      s_SwerveDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      s_SwerveDrive.getKinematics(), // SwerveDriveKinematics
      new PIDConstants(5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      s_SwerveDrive::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      s_SwerveDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    pathGroup = PathPlanner.loadPathGroup("Test5Ball", new PathConstraints(4, 3));

    // Configure the button bindings
    configureButtonBindings();
    Logger.configureLoggingAndConfig(this, false);
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(()->isIntegratedSteering)
      .onFalse(s_SwerveDrive.switchToRemoteSteerCommand().ignoringDisable(true))
      .onTrue(s_SwerveDrive.switchToIntegratedSteerCommand().ignoringDisable(true));
    
    /**
     * Trigger Coast/Brake modes when DS is Disabled/Enabled.
     * Trigger runs WHILETRUE for coast mode.  Coast Mode method
     * is written to wait for slow speeds before setting to coast
     */
    new Trigger(DriverStation::isDisabled)
      .whileTrue(s_SwerveDrive.setToCoast().ignoringDisable(true))
      .onFalse(s_SwerveDrive.setToBrake());

  }



  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    return 
      autoBuilder.fullAuto(pathGroup);    
  }


  @Config
  public void isIntegratedSteering(boolean input){
    isIntegratedSteering = input;
  }

  public double rotationInputController(){
    if(Math.abs(-xBox.getRawAxis(xBoxRot)) < .2 && (xBox.getPOV() > -1)){
      rotationController.setP(Preferences.getDouble("pKPRotationController", SwerveConstants.kpRotationController));
      
      return rotationController.calculate(Math.IEEEremainder(s_SwerveDrive.getRobotAngleDegrees(), 360), xBox.getPOV()-180);
    } else {
      return -xBox.getRawAxis(xBoxRot);
    }
  }


}
