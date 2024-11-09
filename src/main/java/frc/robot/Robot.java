// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private CommandXboxController controller;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    public Robot() {
        controller = new CommandXboxController(0);

        drivetrain = new Drivetrain();
        intake = new Intake();
        shooter = new Shooter();

        drivetrain.setDefaultCommand(
            drivetrain.drive(controller::getLeftX, controller::getRightY)
        );

        controller.leftTrigger().and(intake.gamePieceDetected.negate())
            .whileTrue(intake.feed());

        controller.rightTrigger().and(intake.gamePieceDetected)
            .onTrue(shooter.shoot());

        intake.gamePieceDetected.negate()
            .debounce(0.5)
            .onTrue(shooter.stop());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = drivetrain.drive(() -> 0.5, () -> 0.5)
            .withTimeout(3)
            .andThen(drivetrain.turn(Degrees.of(180)))
            .andThen(drivetrain.drive(() -> 0.5, () -> 0.5)
                .withTimeout(3));

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
