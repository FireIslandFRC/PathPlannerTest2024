package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.BumpOut;
import frc.robot.commands.Intake;
import frc.robot.commands.LowerArm;
import frc.robot.commands.LowerArmFlywheel;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.RotateSouce;
import frc.robot.commands.RotateToAprilTag;
import frc.robot.commands.S_DriveCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAtDistance;
import frc.robot.commands.ShootAtDistanceAuto;
import frc.robot.commands.ShootAtN1;
import frc.robot.commands.ShootAtSafe;
import frc.robot.commands.ShootNoOff;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.SquareUpAmp;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShoot;

public class RobotContainer extends SubsystemBase{
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 
  private final Hand HandSubs = new Hand(); 

  //private final Arm ArmSubs = new Arm(); 
  private final SendableChooser<Command> autoChooser2;

  //SENDABLECHOOSER


  //CONTROLLERS  
  public final static XboxController xbox = new XboxController(ControllerConstants.kOperatorControllerPort);
  public final static Joystick drive = new Joystick(ControllerConstants.kDriverControllerPort);

    //private final XboxController drive = new XboxController(0);

  //DRIVE BUTTONS 
  private final JoystickButton speedButton = new JoystickButton(drive, 1);
  private final JoystickButton fieldOriented = new JoystickButton(drive, 2);
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 3);
  //private final JoystickButton resetPosButton = new JoystickButton(drive, 3);
  private final JoystickButton Rotateright = new JoystickButton(drive, 4);
  private final JoystickButton SquareUpAmpButton = new JoystickButton(drive, 10);
  private final JoystickButton UpClimber = new JoystickButton(drive, 6);
  private final JoystickButton DownClimber = new JoystickButton(drive, 9);
  private final JoystickButton AlignSource = new JoystickButton(drive, 5);
  private final JoystickButton AlignSpeaker = new JoystickButton(drive, 8);
  private final JoystickButton RaiseArm = new JoystickButton(xbox, XboxController.Button.kA.value);
  private final JoystickButton LowerArm = new JoystickButton(xbox, XboxController.Button.kB.value);
  private final JoystickButton Intake = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  private final JoystickButton Intake2 = new JoystickButton(xbox, XboxController.Button.kX.value);
  private final JoystickButton Shoot = new JoystickButton(xbox, 4);
  private final JoystickButton ShootAtDist = new JoystickButton(xbox, 8);
  private final JoystickButton Outtake = new JoystickButton(xbox, 5);
  private final JoystickButton SafeShoot = new JoystickButton(xbox, 7);
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
        () -> fieldOriented.getAsBoolean(),
        () -> speedButton.getAsBoolean()
      )
    );

    ///m_field = new Field2d();
    //SmartDashboard.putData(m_field);
    

    NamedCommands.registerCommand("LowerArmFlywheel", new LowerArmFlywheel());
    NamedCommands.registerCommand("LowerArm", new LowerArm());
    //NamedCommands.registerCommand("StartShoot", new ShootNoOff());
    NamedCommands.registerCommand("ShootAtDistanceAuto", new ShootAtDistanceAuto());
    NamedCommands.registerCommand("ShootAtN1", new ShootAtN1());
    NamedCommands.registerCommand("StopShooter", new StopShoot());
    NamedCommands.registerCommand("ShootNoOff", new ShootNoOff());
    NamedCommands.registerCommand("BumpOut", new BumpOut());
    NamedCommands.registerCommand("StopIntake", new StopIntake());
    NamedCommands.registerCommand("RotateToSpeaker", new RotateToAprilTag(swerveSubs));



    autoChooser2 = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto chooser", autoChooser2);

    


    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    //TODO: all buttons
    //resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry()));
    RaiseArm.whileTrue(new RaiseArm());
    LowerArm.whileTrue(new LowerArm());
    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));
    //resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry()));
    Intake2.whileTrue(new Intake());
    Intake.whileTrue(new SourceIntake());
    Shoot.whileTrue(new Shoot());
    Outtake.whileTrue(new ReverseIntake());
    //Rotateright.whileTrue(new RotateCommand(swerveSubs, 90));
    SquareUpAmpButton.whileTrue(new SquareUpAmp(swerveSubs, () -> drive.getX(), () -> drive.getY()));
    AlignSource.whileTrue(new RotateSouce(swerveSubs, () -> drive.getY(), () -> drive.getX()));
    AlignSpeaker.whileTrue(new RotateToAprilTag(swerveSubs));
    ShootAtDist.whileTrue(new ShootAtDistance());
    UpClimber.whileTrue(new RaiseClimber());
    DownClimber.whileTrue(new LowerClimber());
    SafeShoot.whileTrue(new ShootAtSafe());
    //Ground.whileTrue(new GroundIntake());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser2.getSelected();
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // return AutoBuilder.followPath(path);
  }

  @Override
  public void periodic() {
   //SmartDashboard.putNumber("ArmAngle", ArmSubs.GetArmPos());
    //m_field.setRobotPose(swerveSubs.getPose());
    SmartDashboard.putNumber("Arm Angle", Arm.ArmEncoder.getPosition());
    SmartDashboard.putNumber("ClimerAngleL", Climber.ClimberLeftEncoder.getPosition());
    SmartDashboard.putNumber("ClimerAngleR", Climber.ClimberRightEncoder.getPosition());
  }

}