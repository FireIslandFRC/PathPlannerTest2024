package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class ShootAtDistance extends Command {


  public ShootAtDistance() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.ArmToDistance();
    Hand.ShootAtSpeed(1);
    if (Arm.ArmIsAtPoint()){
      Hand.Intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hand.StopHand();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
}
}
