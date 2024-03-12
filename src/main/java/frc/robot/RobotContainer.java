//2023 SwerveFrame


package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

public class RobotContainer extends SubsystemBase{
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //CONTROLLERS  
  private final Joystick drive = new Joystick(0);
  private final XboxController xbox = new XboxController(1);

  //DRIVE BUTTONS 
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 1); 
  private final JoystickButton resetPosButton = new JoystickButton(xbox, XboxController.Button.kB.value);
  private final JoystickButton MoveToRandomPose = new JoystickButton(xbox, XboxController.Button.kX.value); 

  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;

  //POSE POINTS
  Pose2d RandomPose = new Pose2d(2, 2, Rotation2d.fromDegrees(0));

  //SENDABLECHOOSER
  //private final SendableChooser<Command> autoChooser;

  
  public Field2d m_field;



  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs,
        () -> -drive.getY(),
        () -> -drive.getX(),
        () -> (-drive.getTwist()) - 0.1,
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
    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));
    resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry(new Pose2d())));
    MoveToRandomPose.whileTrue(Commands.deferredProxy(() -> swerveSubs.driveToPose(
      new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0)))
 ));

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
