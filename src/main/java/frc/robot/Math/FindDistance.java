package frc.robot.Math;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class FindDistance {

    public double Dist;

    public FindDistance(){
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
    
}
