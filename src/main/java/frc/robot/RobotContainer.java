package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

public class RobotContainer {
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //CONTROLLERS  
  private final XboxController xbox = new XboxController(ControllerConstants.kDriverControllerPort);

  //DRIVE BUTTONS 
  private final JoystickButton resetNavxButton = new JoystickButton(xbox, XboxController.Button.kA.value); 
  private final JoystickButton quickTurnButton = new JoystickButton(xbox, XboxController.Button.kX.value); 

  private final JoystickButton fullSpeedButton = new JoystickButton(xbox, XboxController.Button.kY.value); 

  //AXIS 
  private final int joystickAxis = XboxController.Axis.kRightY.value;


  public RobotContainer() {
    swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), true));
    // shooter.setDefaultCommand(new Sh_JoystickControlCommand(shooter, () -> xbox.getRawAxis(joystickAxis) * 0.9));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    resetNavxButton.onTrue(new InstantCommand(() -> swerveSubs.resetNavx()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("New Auto");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // return AutoBuilder.followPath(path);
  }

}
