package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;

public class Hand extends SubsystemBase {
  private static CANSparkMax TopShooter = new CANSparkMax(HandConstants.TopShooterID, MotorType.kBrushless);
  private static CANSparkMax BottonShooter = new CANSparkMax(HandConstants.BottomShooterID, MotorType.kBrushless);
  private static CANSparkMax Intake = new CANSparkMax(HandConstants.IntakeID, MotorType.kBrushless);
  public static DigitalInput HandLimitSwitch = new DigitalInput(0);

  public Hand() {
    BottonShooter.setInverted(false);
    TopShooter.setInverted(false);
    BottonShooter.follow(TopShooter);
  }

  public static void ShootAtSpeed(double speed){
    TopShooter.set(speed);
  }

  public static void IdleShoot(){
    TopShooter.set(0.5);
    }

  public static void Intake(){
    Intake.set(0.5);
  }

  public static void ReverseIntake(){
    Intake.set(-0.5);
  }

  public static void StopShooter(){
    TopShooter.set(0);
  }

  public static void StopIntake(){
    Intake.set(0);
  }

  public static void StopHand(){
    Intake.set(0);
    TopShooter.set(0);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", TopShooter.getEncoder().getVelocity());
  }

}
