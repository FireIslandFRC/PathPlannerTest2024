package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.LowerArm;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.S_DriveCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAtDistance;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.SquareUpAmp;

public class RobotContainer extends SubsystemBase{
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 
  //private final Arm ArmSubs = new Arm(); 


  //CONTROLLERS  
  private final XboxController xbox = new XboxController(ControllerConstants.kOperatorControllerPort);
  private final Joystick drive = new Joystick(ControllerConstants.kDriverControllerPort);

    //private final XboxController drive = new XboxController(0);

  //DRIVE BUTTONS 
  private final JoystickButton speedButton = new JoystickButton(drive, 1);
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 2);
  //private final JoystickButton resetPosButton = new JoystickButton(drive, 3);
  private final JoystickButton Rotateright = new JoystickButton(drive, 4);
  private final JoystickButton SquareUpAmpButton = new JoystickButton(drive, 3);
  private final JoystickButton AlignSource = new JoystickButton(drive, 5);
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
        true,
        () -> speedButton.getAsBoolean()
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
    //resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry()));
    RaiseArm.whileTrue(new ShootAtDistance());
    LowerArm.whileTrue(new LowerArm());
    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));
    //resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry()));
    Intake2.whileTrue(new Intake());
    Intake.whileTrue(new SourceIntake());
    Shoot.whileTrue(new Shoot());
    Outtake.whileTrue(new ReverseIntake());
    Rotateright.whileTrue(new RotateCommand(swerveSubs, 90));
    SquareUpAmpButton.whileTrue(new SquareUpAmp(swerveSubs, () -> drive.getY()));
    AlignSource.whileTrue(new RotateCommand(swerveSubs, 60));
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
   //SmartDashboard.putNumber("ArmAngle", ArmSubs.GetArmPos());
    //m_field.setRobotPose(swerveSubs.getPose());
  }

}