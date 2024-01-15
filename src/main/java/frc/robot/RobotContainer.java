// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.TeleopSwerve;
import frc.robot.sensors.NavX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OperatorSystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.OperatorSystem.JoystickConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem swerve = new DriveSubsystem();
  private final OperatorSystem OI = new OperatorSystem();
  private final VisionSubsystem limelight = new VisionSubsystem();
  private final NavX navX = new NavX();
  private final PoseEstimationSubsystem poseEstimationSubsystem = new PoseEstimationSubsystem(swerve, limelight, navX);
  
  PS4Controller driverController = OI.driverController;
  Joystick operatorController = OI.operatorController;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    swerve.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new TeleopSwerve(
            swerve,
            () -> (driverController.getRawAxis(JoystickConstants.translationAxis)),
            () -> (driverController.getRawAxis(JoystickConstants.strafeAxis)),
            () -> (driverController.getRawAxis(JoystickConstants.rotAxis)),
            () -> true
        )
    );
    
  }

  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    return new WaitCommand(1);
  }

}
