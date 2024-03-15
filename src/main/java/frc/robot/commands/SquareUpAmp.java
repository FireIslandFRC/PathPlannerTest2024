package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class SquareUpAmp extends Command {
  private SwerveSubsystem swerveSubs; 
  private PIDController StrafePID, RotationPID; 
  private DoubleSupplier yController;

  public SquareUpAmp(SwerveSubsystem swerveSubs, DoubleSupplier yController) {
    this.swerveSubs = swerveSubs; 
    this.yController = yController;
    RotationPID = new PIDController(0.005, 0, 0);
    StrafePID = new PIDController(0.005, 0, 0);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double StrafeSpeed, RotationSpeed;

    StrafeSpeed = StrafePID.calculate(LimelightHelpers.getTX("limelight"), 0);
    RotationSpeed = RotationPID.calculate(swerveSubs.getRotation2d().getDegrees(), 90);
    //double[] PoseFinder = LimelightHelpers.getCameraPose_TargetSpace("limelight");

    swerveSubs.drive(StrafeSpeed, yController.getAsDouble(), RotationSpeed, true, 1);

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
