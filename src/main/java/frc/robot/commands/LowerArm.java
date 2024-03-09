package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.StopArm();
    Arm.LockArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

}
