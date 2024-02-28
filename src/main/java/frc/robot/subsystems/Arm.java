package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
    public static double speed;
    private static CANSparkMax Arm_Motor = new CANSparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);
    private static DoubleSolenoid Brake = new DoubleSolenoid(Constants.PhID, PneumaticsModuleType.REVPH, ArmConstants.BrakeID0, ArmConstants.BrakeID1);
    //public static AbsoluteEncoder ArmEncoder = Arm_Motor.getAbsoluteEncoder(Type.kDutyCycle);
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;

    public static RelativeEncoder ArmEncoder;
    private static SwerveSubsystem s_SwerveSubsystem;


    public Arm() {
        ArmEncoder = Arm_Motor.getAlternateEncoder(kAltEncType, kCPR);
        Constants.ArmAngleAtDis.put(4.0, 1.0);
        Constants.ArmAngleAtDis.put(4.5, 2.0);
        Constants.ArmAngleAtDis.put(5.0, 3.0);
        Constants.ArmAngleAtDis.put(5.5, 4.0);
        Constants.ArmAngleAtDis.put(6.0, 5.0);
        Constants.ArmAngleAtDis.put(6.5, 6.0);
        Constants.ArmAngleAtDis.put(7.0, 7.0);
        Constants.ArmAngleAtDis.put(7.5, 8.0);
        Constants.ArmAngleAtDis.put(8.0, 9.0);
        Constants.ArmAngleAtDis.put(8.5, 10.0);
        Constants.ArmAngleAtDis.put(9.0, 11.0);
        Constants.ArmAngleAtDis.put(9.5, 12.0);
        Constants.ArmAngleAtDis.put(10.0, 13.0);

        s_SwerveSubsystem = new SwerveSubsystem();
    }

    public static void RaiseArm(){
        Arm_Motor.set(1);
    } 

    public static void LowerArm(){
        Arm_Motor.set(-1);
    }

    public static boolean ArmToPoint(double pos){
        speed = 0.1 * (pos - ArmEncoder.getPosition());
        if ((pos - ArmEncoder.getPosition()) > 0.1 || (pos - ArmEncoder.getPosition()) < 0.1){
            Arm_Motor.set(speed);
        }else{
            Arm_Motor.set(0);
            return true;
        }
        return false;
    }

    public static void LockArm(){
        Brake.set(Value.kForward);
    }

    public static void UnLockArm(){
        Brake.set(Value.kReverse);
    }

    public static void StopArm(){
        Arm_Motor.set(0);
    }

    public double GetArmPos(){
        ArmEncoder.getPosition();
        return ArmEncoder.getPosition();
    }

    public static void DesiredArmAngle(){
        
    }


    @Override
    public void periodic() {
       SmartDashboard.putNumber("Arm pos", GetArmPos());
    }

}
