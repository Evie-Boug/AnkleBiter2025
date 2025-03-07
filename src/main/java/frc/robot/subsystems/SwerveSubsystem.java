package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.encoders.SwerveAbsoluteEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

import swervelib.SwerveController;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  public final double maximumSpeed = Constants.SwerveConstants.MAXIMUM_SPEED;

  public SwerveSubsystem(File directory) {
    try {
      swerveDrive = new swervelib.parser.SwerveParser(directory).createSwerveDrive(3);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    for (SwerveModule module : swerveDrive.getModules()) {
      module.setAntiJitter(false);
    }
    swerveDrive.setCosineCompensator(false);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void resetOdometry(Pose2d initialPose) {
    swerveDrive.resetOdometry(initialPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void stop() {
    drive(new Translation2d(0, 0), 0, false);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(() -> {
      swerveDrive.drive(
        new Translation2d(translationX.getAsDouble() * maximumSpeed, 
                          translationY.getAsDouble() * maximumSpeed),
        rotation.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        true, false
      );
    });
  }

  public void updateSmartDashboard() {
    int moduleIndex = 0;
    for (SwerveModule module : swerveDrive.getModules()) {
        SwerveAbsoluteEncoder canCoder = module.getAbsoluteEncoder();
        double position = canCoder.getAbsolutePosition();
        SmartDashboard.putNumber("Module " + moduleIndex + " Encoder Position", position);
        moduleIndex++;
    }
}

  @Override
  public void periodic() {
    int moduleIndex = 0;
    for (SwerveModule module : swerveDrive.getModules()) {
        SwerveAbsoluteEncoder canCoder = module.getAbsoluteEncoder();
        double position = canCoder.getAbsolutePosition();
        SmartDashboard.putNumber("Module " + moduleIndex + " Encoder Position", position);
        moduleIndex++;
    }
  }
    
    public SwerveController getSwerveController() {
      
      return swerveDrive.swerveController;
  }
}
