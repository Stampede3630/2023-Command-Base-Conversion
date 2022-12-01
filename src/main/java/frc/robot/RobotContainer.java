// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveDrive;
import io.github.oblarg.oblog.Logger;


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

  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      Preferences.setBoolean("pFieldRelative", Constants.fieldRelative);
      Preferences.setBoolean("pAccelInputs", Constants.acceleratedInputs);
      Preferences.setDouble("pDriveGovernor", Constants.driveGovernor);

      s_SwerveDrive.setDefaultCommand( 
        s_SwerveDrive.joystickDriveCommand(
            () -> -xBox.getRawAxis(xBoxXAxis),
            () -> -xBox.getRawAxis(xBoxYAxis),
            () -> -xBox.getRawAxis(xBoxRot),
            () -> Preferences.getDouble("pDriveGovernor", Constants.driveGovernor),
            () -> Preferences.getBoolean("pFieldRelative", Constants.fieldRelative),
            () -> Preferences.getBoolean("pAccelInputs", Constants.acceleratedInputs)
            )
      );
            
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
  private void configureButtonBindings() {}

  

}
