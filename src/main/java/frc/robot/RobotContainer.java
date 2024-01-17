package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

public class RobotContainer {
  //SUBSYSTEMS 
  SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //CONTROLLERS  
  XboxController xbox = new XboxController(ControllerConstants.kDriverControllerPort);

  //DRIVE BUTTONS 
  JoystickButton resetNavxButton = new JoystickButton(xbox, XboxController.Button.kA.value); 
  JoystickButton quickTurnButton = new JoystickButton(xbox, XboxController.Button.kX.value); 

  public RobotContainer() {
    swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -xbox.getLeftX(), () -> xbox.getLeftY(), () -> -xbox.getRightX(), false));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
