package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.S_DriveCommand;

public class RobotContainer extends SubsystemBase{
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //CONTROLLERS  
  private final XboxController xbox = new XboxController(ControllerConstants.kOperatorControllerPort);
  private final Joystick joystick = new Joystick(ControllerConstants.kDriverControllerPort);
  
  //DRIVE BUTTONS 
  private final JoystickButton resetPigeonButton = new JoystickButton(joystick, 1); 
  private final JoystickButton resetPosButton = new JoystickButton(joystick, 2);
  private final JoystickButton RaiseArm = new JoystickButton(xbox, XboxController.Button.kA.value); 
  private final JoystickButton LowerArm = new JoystickButton(xbox, XboxController.Button.kB.value); 

  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  public Field2d m_field;


  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs,
        () -> -xbox.getLeftY(),
        () -> -xbox.getLeftX(),
        () -> -xbox.getRightX(),
        true
      )
    );

    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("auto chooser", autoChooser);


    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    //TODO: all buttons
    resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry()));
    RaiseArm.whileTrue(new RaiseArm());
    LowerArm.whileTrue(new LowerArm());
    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));
    resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry()));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("New New Auto");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // return AutoBuilder.followPath(path);
  }

  @Override
  public void periodic() {
        m_field.setRobotPose(swerveSubs.getPose());

  }

}
