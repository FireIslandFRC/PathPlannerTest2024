package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateSouce extends Command {
  private SwerveSubsystem swerveSubs; 
  private PIDController rotationPID; 
  private DoubleSupplier x, y;
  private double setpoint; 
  

  public RotateSouce(SwerveSubsystem swerveSubs, DoubleSupplier x, DoubleSupplier y) {
    this.swerveSubs = swerveSubs; 
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

    if (Robot.ally.isPresent()) {
      if (Robot.ally.get() == Alliance.Red) {
          setpoint = 60;
          System.out.println("Red");
      }
      if (Robot.ally.get() == Alliance.Blue) {
          setpoint = 300;
          System.out.println("Blue");
      }
    }
    else {
      setpoint = 0;
      System.out.println("Null");
    }

    rotationSpeed = rotationPID.calculate(swerveSubs.getRotation2d().getDegrees(), setpoint);

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
