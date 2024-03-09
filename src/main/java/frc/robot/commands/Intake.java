package frc.robot.commands;

import com.revrobotics.jni.RevJNIWrapper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class Intake extends Command {

    private boolean done = false;
    private Timer PickUpTimer = new Timer();

  public Intake() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PickUpTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Hand.Intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hand.StopIntake();
    //Hand.IdleShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    ////Hand.StopIntake();
    //Hand.IdleShoot();
    return done;
  }

}
