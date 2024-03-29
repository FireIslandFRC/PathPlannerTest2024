package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hand;

public class BumpOut extends Command {

  private Timer bumpout = new Timer();

  private boolean done = false;

  public BumpOut() {
    bumpout.restart();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Hand.ShootAtSpeed(0.9);
    if(bumpout.get() < 0.7){
      Hand.ReverseIntake();
    }else{  
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hand.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

}
