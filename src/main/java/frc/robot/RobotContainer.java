// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtAmp;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.LooseHook;
import frc.robot.commands.HangOnStage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hang;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController pilotJoystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Pitch pitch = new Pitch();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Hang hang = new Hang();
  
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-pilotJoystick.getRightY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-pilotJoystick.getRightX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-pilotJoystick.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    pilotJoystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    pilotJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    /* intake commands */
    pilotJoystick.leftTrigger(0.5).whileTrue(intake.runIntakeUntilNotePresent(pilotJoystick.getHID()));
    pilotJoystick.a().whileTrue(Commands.run(intake::runReverse, intake));

    /* shooter commands */
    pilotJoystick.b().whileTrue(new AimAtSpeaker(pitch, shooter));
    pilotJoystick.y().whileTrue(new AimAtAmp(pitch, shooter));
    pilotJoystick.rightTrigger(0.5).whileTrue(Commands.run(intake::runIdle, intake));

    /* hang commands */
    pilotJoystick.button(5).whileTrue(new LooseHook(hang));
    pilotJoystick.button(6).whileTrue(new HangOnStage(hang));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
