package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class LowerArm extends Command {

  private boolean done = false;

  public LowerArm() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.UnLockArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.LowerArm();
    if(Arm.ArmEncoder.getPosition() < 5){
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

}
