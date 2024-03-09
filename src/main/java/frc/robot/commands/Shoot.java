package frc.robot.commands;

import com.revrobotics.jni.RevJNIWrapper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;

public class Shoot extends Command {

    private boolean done = false;
    private Timer RevTime = new Timer();

  public Shoot() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RevTime.reset();
    RevTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Hand.ShootAtSpeed(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hand.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

}
