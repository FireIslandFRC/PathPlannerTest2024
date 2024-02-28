package frc.robot;

import java.lang.Math;

public class MathFuncs {

    public double Dist;

    public MathFuncs(){
    }

    public double Calculate(double CurrentX, double CurrentY){
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
    
}
