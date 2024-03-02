package frc.robot;

import java.lang.Math;
import frc.robot.subsystems.SwerveSubsystem;

public class MathFuncs {

    public double Dist;
    private double CurrentX, CurrentY;
    private SwerveSubsystem s_SwerveSubsystem;
    private double CalcAngle, WantedAngle;

    public MathFuncs(){
        CurrentX = s_SwerveSubsystem.getPose().getX();
        CurrentY = s_SwerveSubsystem.getPose().getY();
    }

    public double CalculateDistance(){
        Dist = (
            Math.sqrt(
                Math.pow(
                    (CurrentX - 9), (2)
                ) + Math.pow(
                    (CurrentY - 0), (2)
                )
            )
        );
        return Dist;
    }

    public double RoundToHalf(double num){
        return (  (  Math.round(num * 2)  ) / 2   );
    }

    public double ShootingAngle(){
        CalcAngle = Math.atan((9-CurrentX) / CurrentY);
        WantedAngle = 180 - CalcAngle;
        return WantedAngle;
    }
    
}
