package frc.robot;

import java.util.concurrent.SynchronousQueue;

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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.Intake;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.S_DriveCommand;
import frc.robot.commands.Shoot;

public class RobotContainer extends SubsystemBase{
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //CONTROLLERS  
  private final XboxController xbox = new XboxController(ControllerConstants.kOperatorControllerPort);
  private final Joystick drive = new Joystick(ControllerConstants.kDriverControllerPort);

    //private final XboxController drive = new XboxController(0);

  //DRIVE BUTTONS 
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 1);
  private final JoystickButton resetPosButton = new JoystickButton(drive, 2);
  private final JoystickButton RaiseArm = new JoystickButton(xbox, XboxController.Button.kA.value);
  private final JoystickButton LowerArm = new JoystickButton(xbox, XboxController.Button.kB.value);
  private final JoystickButton Intake = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  private final JoystickButton Intake2 = new JoystickButton(xbox, XboxController.Button.kX.value);
  private final JoystickButton Shoot = new JoystickButton(xbox, 4);
  private final JoystickButton Outtake = new JoystickButton(xbox, 5);
  //private final JoystickButton Ground = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  //public Field2d m_field;


  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs,
        () -> -drive.getY(),
        () -> -drive.getX(),
        () -> -drive.getTwist(),
        true
      )
    );

    ///m_field = new Field2d();
    //SmartDashboard.putData(m_field);

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
    Intake.whileTrue(new Intake());
    Intake2.whileTrue(new Intake());
    Shoot.whileTrue(new Shoot());
    Outtake.whileTrue(new ReverseIntake());
    //Ground.whileTrue(new GroundIntake());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Taxi");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // return AutoBuilder.followPath(path);
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("ArmAngle", Arm.GetArmPos());
    //m_field.setRobotPose(swerveSubs.getPose());
  }

}