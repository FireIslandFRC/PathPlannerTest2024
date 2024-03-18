package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class RaiseClimber extends Command {

    private boolean done = false;

  public RaiseClimber() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Arm.UnLockArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Climber.RaiseClimberLeft();
    Climber.RaiseClimberRight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber.StopClimberLeft();
    Climber.StopClimberRight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
