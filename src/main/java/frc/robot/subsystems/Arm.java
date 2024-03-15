package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MathFuncs;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
    public static double speed;
    private static CANSparkMax Arm_Motor = new CANSparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);
    private static CANSparkMax Arm_MotorL = new CANSparkMax(ArmConstants.ArmMotorLID, MotorType.kBrushless);
    public static AbsoluteEncoder ArmEncoder = Arm_Motor.getAbsoluteEncoder();

    //public static RelativeEncoder ArmEncoder;
    private static MathFuncs math;
    private static double distance;
    private static double roundedDistance;

    public static HashMap<Double, Double> ArmAngleAtDis = new HashMap<Double, Double>();



    public Arm() {
        Arm_MotorL.setInverted(true);
        //ArmEncoder = Arm_Motor.getAlternateEncoder(kAltEncType, kCPR);
        ArmAngleAtDis.put(3.0, 4.0);
        ArmAngleAtDis.put(4.0, 10.0);
        ArmAngleAtDis.put(5.0, 14.0);
        ArmAngleAtDis.put(6.0, 20.0);
        ArmAngleAtDis.put(7.0, 25.0);
        ArmAngleAtDis.put(8.0, 27.0);
        ArmAngleAtDis.put(9.0, 29.5);

        math = new MathFuncs();
    }

    public static void RaiseArm(){
        if (ArmEncoder.getPosition() < 100){
             Arm_Motor.set(0.5);
            Arm_MotorL.set(-0.35);
            System.out.println(ArmAngleAtDis.get(5.0));
        }else {
            Arm_Motor.set(0);
            Arm_MotorL.set(-0);
        }
    } 

    public static void LowerArm(){
        if (ArmEncoder.getPosition() > 8
        ){
            Arm_Motor.set(-0.2);
            Arm_MotorL.set(0.2);
        }else{
            Arm_Motor.set(0);
            Arm_MotorL.set(0);
        }
    }


    public static boolean ArmToPoint(double pos){
        boolean finished;
        if ( ArmEncoder.getPosition() < pos){
            Arm_Motor.set(0.35);
            Arm_MotorL.set(-0.35);
            finished = false;
        }else if(ArmEncoder.getPosition() > pos + 2){
            Arm_Motor.set(-0.2);
            Arm_MotorL.set(0.2);
            finished = false;
        }else{
            Arm_Motor.set(0);
            Arm_MotorL.set(0);
            finished = true;
        }
        return finished;
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

    /*public double GetArmPos(){
        return ArmEncoder.getPosition();
    }*/

    public static void ArmToDistance(){
        if (ArmEncoder.getPosition() < ArmAngleAtDis.get(5.0)){
            Arm_Motor.set(0.35);
            Arm_MotorL.set(-0.35);
        }else if(ArmEncoder.getPosition() > ArmAngleAtDis.get(5.0)){
            Arm_Motor.set(-0.2);
            Arm_MotorL.set(0.2);
        }else{
            Arm_Motor.set(0);
            Arm_MotorL.set(0);
        }
    }

    public static boolean ArmIsAtPoint(){
        boolean AtPoint;
        if(ArmEncoder.getPosition() == ArmAngleAtDis.get(5.0)){
            AtPoint = true;
        }else{
            AtPoint = false;
        }
        return AtPoint;
    }

    public static double DesiredArmAngle(){
        distance = math.CalculateDistance();
        roundedDistance = math.RoundToHalf(distance);
        return ArmAngleAtDis.get(roundedDistance);
    }

    @Override
    public void periodic() {
       SmartDashboard.putNumber("Arm pos", ArmEncoder.getPosition());
    }

}