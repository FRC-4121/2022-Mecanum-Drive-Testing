// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Seth was here so was francesco
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithJoysticks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
//this is comment -- i keep chaning this
public class RobotContainer {

  //driver controllers
  private final XboxController xbox = new XboxController(XBOX_PORT);
  
  //subsystems
  private Drivetrain drivetrain = new Drivetrain();
  
  //commands
  final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, xbox);

 


  //this constructor runs the methods below
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
    
  }







  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/

    //For subsystem default commands (driving, etc.)
    private void configureDefaultCommands(){

      //Drivetrain -> drive with xbox joysticks
      drivetrain.setDefaultCommand(driveCommand);
    }
}