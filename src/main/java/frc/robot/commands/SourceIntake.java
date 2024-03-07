package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class SourceIntake extends Command {

    private boolean done = false;

  public SourceIntake() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.UnLockArm();
    Hand.IntakeUntilSwitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.ArmToPoint(0);
    if (Arm.ArmToPoint(0)){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.StopArm();
    Arm.LockArm();
    done = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Arm.StopArm();
    Arm.LockArm();
    return done;
  }

}
