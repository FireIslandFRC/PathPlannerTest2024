package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;;

public class Climber extends SubsystemBase {

  public static CANSparkMax ClimberLeft = new CANSparkMax(ClimberConstants.LeftClimber , MotorType.kBrushless);
  public static CANSparkMax ClimberRight = new CANSparkMax(ClimberConstants.RightClimber , MotorType.kBrushless);
  public static RelativeEncoder ClimberRightEncoder = ClimberRight.getEncoder();
  public static RelativeEncoder ClimberLeftEncoder = ClimberLeft.getEncoder();
  //public static DoubleSolenoid BrakeLeft = new DoubleSolenoid(Constants.PhID, PneumaticsModuleType.REVPH, ClimberConstants.LockLeft, ClimberConstants.UnLockLeft);
  //public static DoubleSolenoid BrakeRight = new DoubleSolenoid(Constants.PhID, PneumaticsModuleType.REVPH, ClimberConstants.LockRight, ClimberConstants.UnLockRight);

  public Climber() {

  }

  public static void RaiseClimberLeft(){
    if (ClimberLeftEncoder.getPosition() < 238){
    ClimberLeft.set(0.7);
    }else{
      ClimberLeft.set(0);
    }
  }

  public static void LowerClimberLeft(){
    if (ClimberLeftEncoder.getPosition() > 20){
    ClimberLeft.set(-0.5);
    }else{
      ClimberLeft.set(0);
    }
  }

  public static void RaiseClimberRight(){
    if (ClimberRightEncoder.getPosition() > -236){
    ClimberRight.set(-0.7);
    }else{
      ClimberRight.set(0);
    }
  }

  public static void LowerClimberRight(){
    if (ClimberRightEncoder.getPosition() < -20){
      ClimberRight.set(0.5);
    }else{
      ClimberRight.set(0);
    }
  }

  public static void Lock(){
    //BrakeLeft.set(Value.kForward);
    //BrakeRight.set(Value.kForward);
  }

  public static void UnLock(){
    //BrakeLeft.set(Value.kReverse);
    //BrakeRight.set(Value.kReverse);
  }

  public static void StopClimberLeft(){
    ClimberLeft.set(0);
  }

  public static void StopClimberRight(){
    ClimberRight.set(0);
  }

  @Override
  public void periodic() {
    
  }

}
