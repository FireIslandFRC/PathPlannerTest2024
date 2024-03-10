package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class GroundIntake extends Command {

    private boolean done = false;

  public GroundIntake() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.UnLockArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.ArmToPoint(10);
    Hand.Intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.StopArm();
    Hand.StopHand();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

}
