package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class ShootAtDistanceAuto extends Command {

  private Timer bumpTimer = new Timer();
  private boolean bumpBool = false;
  private boolean done = false;

  public ShootAtDistanceAuto() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bumpTimer.restart();
    //Hand.ReverseIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (bumpTimer.get() > 0.2 && !bumpBool){
      Hand.StopIntake();
      bumpBool = true;
    }*/
    Hand.ShootAtSpeed(1);
    Arm.ArmToDistance(2);
    if(Arm.ArmEncoder.getPosition() > 3){
      done = true;
    }else{
      done = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.StopArm();
  }

  // Return s true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
