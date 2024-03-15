package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AmpLineUp extends Command {
  private SwerveSubsystem swerveSubs; 
  private PIDController StrafePID; 

  public AmpLineUp(SwerveSubsystem swerveSubs) {
    this.swerveSubs = swerveSubs; 
    StrafePID = new PIDController(0.01, 0, 0);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double StrafeSpeed;

    StrafeSpeed = StrafePID.calculate(LimelightHelpers.getTX("limelight"), 0);

    swerveSubs.drive(StrafeSpeed, 0, 0, true, 1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
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
