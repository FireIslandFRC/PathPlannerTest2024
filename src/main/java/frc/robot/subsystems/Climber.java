package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;;

public class Climber extends SubsystemBase {

  public static CANSparkMax ClimberLeft = new CANSparkMax(ClimberConstants.LeftClimber , MotorType.kBrushless);
  public static CANSparkMax ClimberRight = new CANSparkMax(ClimberConstants.RightClimber , MotorType.kBrushless);
  public static DoubleSolenoid BrakeLeft = new DoubleSolenoid(Constants.PhID, PneumaticsModuleType.REVPH, ClimberConstants.LockLeft, ClimberConstants.UnLockLeft);
  public static DoubleSolenoid BrakeRight = new DoubleSolenoid(Constants.PhID, PneumaticsModuleType.REVPH, ClimberConstants.LockRight, ClimberConstants.UnLockRight);

  public Climber() {

  }

  public static void RaiseClimberLeft(){
    ClimberLeft.set(1);
  }

  public static void LowerClimberLeft(){
    ClimberLeft.set(-1);
  }

  public static void RaiseClimberRight(){
    ClimberRight.set(1);
  }

  public static void LowerClimberRight(){
    ClimberRight.set(-1);
  }

  public static void Lock(){
    BrakeLeft.set(Value.kForward);
    BrakeRight.set(Value.kForward);
  }

  public static void UnLock(){
    BrakeLeft.set(Value.kReverse);
    BrakeRight.set(Value.kReverse);
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
