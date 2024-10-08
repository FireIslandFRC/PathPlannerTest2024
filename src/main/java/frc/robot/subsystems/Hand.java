package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.HandConstants;

public class Hand extends SubsystemBase {
  private static CANSparkMax TopShooter = new CANSparkMax(HandConstants.BottomShooterID, MotorType.kBrushed);
  private static CANSparkMax BottonShooter = new CANSparkMax(HandConstants.TopShooterID, MotorType.kBrushed);
  private static CANSparkMax Intake = new CANSparkMax(HandConstants.IntakeID, MotorType.kBrushless);
  public static DigitalInput HandLimitSwitch = new DigitalInput(0);
  public static final XboxController xbox = new XboxController(ControllerConstants.kOperatorControllerPort);

  public Hand() {
    Intake.setInverted(true);
  }

  public static void ShootAtSpeed(double speed){
    TopShooter.set(-speed);
    BottonShooter.set(-speed);
  }

  public static void IdleShoot(){
    TopShooter.set(0.5);
    }

  public static void Intake(){
    Intake.set(0.5);
    /*if(HandLimitSwitch.get()){
      xbox.setRumble(RumbleType.kBothRumble, 1);
    }else if(!HandLimitSwitch.get()){
      xbox.setRumble(RumbleType.kBothRumble, 0);
    }*/
  }

public static void IntakeSlow(){
    Intake.set(0.2);
    /*if(HandLimitSwitch.get()){
      xbox.setRumble(RumbleType.kBothRumble, 1);
    }else if(!HandLimitSwitch.get()){
      xbox.setRumble(RumbleType.kBothRumble, 0);
    }*/
  }
  public static void ReverseIntake(){
    Intake.set(-0.08

    );
    TopShooter.set(
    0.3
    );
    BottonShooter.set(0.3);
  }

  public static void StopShooter(){
    TopShooter.set(0);
    BottonShooter.set(0);
  }

  public static void StopOnlyIntake(){
    Intake.set(0);
  }

  public static void StopIntake(){
    Intake.set(0);
    TopShooter.set(0);
  BottonShooter.set(0);
  }

  public static boolean IntakeLimit(){
    boolean IntakeSuccess = HandLimitSwitch.get();
    return IntakeSuccess;
  }

  public static void IntakeUntilSwitch(){
    if (!IntakeLimit()){
      Intake();
    }else{
      StopIntake();
    }
  }

  public static void Buzz(){
    RobotContainer.xbox.setRumble(RumbleType.kBothRumble, 1);
  }

  public static void UnBuzz(){
    RobotContainer.xbox.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Shooter Speed", TopShooter.getEncoder().getVelocity());
  }

}
