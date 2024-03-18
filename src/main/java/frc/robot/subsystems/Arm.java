package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MathFuncs;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Intake;


public class Arm extends SubsystemBase {
    public static double speed;
    private static CANSparkMax Arm_Motor = new CANSparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);
    private static CANSparkMax Arm_MotorL = new CANSparkMax(ArmConstants.ArmMotorLID, MotorType.kBrushless);
    public static AbsoluteEncoder ArmEncoder = Arm_Motor.getAbsoluteEncoder();

    //public static RelativeEncoder ArmEncoder;
    private static MathFuncs math;
    private static double distance;
    private static double roundedDistance;




    public Arm() {
        Arm_MotorL.setInverted(true);
        //ArmEncoder = Arm_Motor.getAlternateEncoder(kAltEncType, kCPR);
        

        math = new MathFuncs();
    }

    public static void RaiseArm(){
        if (ArmEncoder.getPosition() < 100){
             Arm_Motor.set(0.5);
            Arm_MotorL.set(-0.35);
            System.out.println(Robot.ArmAngleAtDis.get(4.0));
        }else {
            Arm_Motor.set(0);
            Arm_MotorL.set(-0);
        }
    } 

    public static void LowerArm(){
        if (ArmEncoder.getPosition() > 2){
            Arm_Motor.set(-0.2);
            Arm_MotorL.set(0.2);
        }else{
            Arm_Motor.set(0);
            Arm_MotorL.set(0);
        }
    }


    public static void ArmToPoint(double pos){
        if ( ArmEncoder.getPosition() < pos){
            Arm_Motor.set(0.35);
            Arm_MotorL.set(-0.35);
        }else{
            Arm_Motor.set(0);
            Arm_MotorL.set(0);
        }
    }

    public static void LockArm(){
        //Brake.set(Value.kForward);
    }

    public static void UnLockArm(){
        //Brake.set(Value.kReverse);
    }

    public static void StopArm(){
        Arm_Motor.set(0);
        Arm_MotorL.set(0);

    }

    public double GetArmPos(){
        return ArmEncoder.getPosition();
    }

    public static void ArmToDistance(double dist){
        if (ArmEncoder.getPosition() < dist/*Robot.ArmAngleAtDis.get(8.0)*/){
            Arm_Motor.set(0.35);
            Arm_MotorL.set(-0.35);
        }else{
            Arm_Motor.set(0);
            Arm_MotorL.set(0);
            Hand.IntakeSlow();
    }
    }

    public static double DesiredArmAngle(){
        distance = math.CalculateDistance();
        roundedDistance = math.RoundToHalf(distance);
        return Robot.ArmAngleAtDis.get(roundedDistance);
    }

    public static boolean ArmIsAtPoint(double point){
        boolean IsAtPoint;
        if(ArmEncoder.getPosition() == point){
            IsAtPoint = true;
        }else{
            IsAtPoint = false;
        }
        return IsAtPoint;
    }

    @Override
    public void periodic() {
       SmartDashboard.putNumber("Arm pos", ArmEncoder.getPosition());
    }
}