package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateCommand extends Command {
  private SwerveSubsystem swerveSubs; 
  private PIDController rotationPID; 
  private double angle;
  private DoubleSupplier x, y;

  public RotateCommand(SwerveSubsystem swerveSubs, double angle, DoubleSupplier x, DoubleSupplier y) {
    this.swerveSubs = swerveSubs; 
    this.angle = angle; 
    this.x = x;
    this.y = y;
    rotationPID = new PIDController(0.01, 0, 0);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;

    rotationSpeed = rotationPID.calculate(swerveSubs.getRotation2d().getDegrees(), angle);

    swerveSubs.drive(-x.getAsDouble(), -y.getAsDouble(), rotationSpeed, true, 0.6);

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
