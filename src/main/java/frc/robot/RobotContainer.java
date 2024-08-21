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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.AimAtAmp;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.TogglePitchLock;
import frc.robot.commands.LooseHook;
import frc.robot.commands.NoteStucked;
import frc.robot.commands.HangOnStage;
import frc.robot.commands.PitchUp;
import frc.robot.commands.PitchDown;
import frc.robot.commands.ShootAtAmp;
import frc.robot.commands.ShootAtSpeaker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hang;

public class RobotContainer {
  private final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandJoystick pilotJoystick = new CommandJoystick(0);//Pilot Joystick
  private final CommandXboxController copilotJoystick = new CommandXboxController(1);// Xbox controller
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
        drivetrain.applyRequest(() -> drive.withVelocityX(-pilotJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-pilotJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(pilotJoystick.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    pitch.setDefaultCommand(Commands.run(() -> pitch.runSetPointManually((copilotJoystick.getRightY() * Math.toRadians(90)) + Pitch.LOWEST_POSITION), pitch));

    copilotJoystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    copilotJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    /* intake commands */
    copilotJoystick.leftTrigger(0.5).whileTrue(intake.runIntakeUntilNotePresent(copilotJoystick.getHID()));
    copilotJoystick.a().whileTrue(Commands.run(intake::runReverse, intake));

    /* fix-angled shooter commands */
    copilotJoystick.b().whileTrue(new AimAtSpeaker(pitch, shooter));
    copilotJoystick.y().whileTrue(new AimAtAmp(pitch, shooter));
    copilotJoystick.rightTrigger(0.5).whileTrue(Commands.run(intake::runIdle, intake));

    /* hang commands */
    //copilotJoystick.button(7).whileTrue(new LooseHook(hang));
    //copilotJoystick.button(8).whileTrue(new HangOnStage(hang));

    /* pitch up and down commands */
    //copilotJoystick.button(5).whileTrue(new PitchUp(pitch));
    //copilotJoystick.button(6).whileTrue(new PitchDown(pitch));

    /* loose-angled shooter commands */
    //copilotJoystick.button(10).whileTrue(new ShootAtAmp(shooter));
    //copilotJoystick.button(11).whileTrue(new ShootAtSpeaker(shooter));

    /* toggle lock pitch  commands */
    //copilotJoystick.button(12).whileTrue(new TogglePitchLock(pitch));

    /* note stucked commands */
    copilotJoystick.button(5).whileTrue(new NoteStucked(pitch, intake));

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
