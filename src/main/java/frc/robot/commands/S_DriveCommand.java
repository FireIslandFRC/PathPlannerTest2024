package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class S_DriveCommand extends Command {
  private SwerveSubsystem swerveSubs; 

  private DoubleSupplier xSupplier, ySupplier, zSupplier; 
  private boolean fieldOriented;
  private double SpeedMultiplier;
  private BooleanSupplier speedIncrease;
  /* * * CONSTRUCTOR * * */
  /* 
   * @param swerveSubs the swerve subsystem 
   * @param xSupplier value input for strafe on x-axis 
   * @param ySupplier value input for strafe on y-axis 
   * @param zSupplier value input for rotation 
   * @param fieldOriented whether or not we want the bot to run in field oriented 
   */
  public S_DriveCommand(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented, BooleanSupplier speedIncrease) {
    this.swerveSubs = swerveSubs; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.zSupplier = zSupplier; 
    this.fieldOriented = fieldOriented;
    this.speedIncrease = speedIncrease;
    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* * * ALTERING VALUES * *   */
    //Joystick values -> double 
    double xSpeed = xSupplier.getAsDouble(); 
    double ySpeed = ySupplier.getAsDouble(); 
    double zSpeed = zSupplier.getAsDouble(); 

    SmartDashboard.putNumber("z speed", zSpeed);


    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed); 

    //square the speed values to make for smoother acceleration 
    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

    if (speedIncrease.getAsBoolean()) {
      SpeedMultiplier = 2;
    }else{
      SpeedMultiplier = 1;
    }

    /* * * SETTING SWERVE STATES * * */
    swerveSubs.drive(xSpeed, ySpeed, zSpeed, fieldOriented, SpeedMultiplier);
    
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

/* * * ADDED METHODS * * */
public double deadzone(double num){
    return Math.abs(num) > 0.14 ? num : 0;
}

private static double modifyAxis(double num) {
  // Square the axis
  num = Math.copySign(num * num, num);

  return num;
}


}
