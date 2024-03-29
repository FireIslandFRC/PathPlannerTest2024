package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class ShootAtDistance extends Command {


  public ShootAtDistance() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.ArmToDistance(2.2);
    Hand.ShootAtSpeed(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hand.StopIntake();
    Arm.StopArm();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Arm.ArmIsAtPoint(4);
  }
}
