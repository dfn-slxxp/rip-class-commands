package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SuperstructureDefaultCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Superstructure superstructure;

    public SuperstructureDefaultCommand() {
        swerve = CommandSwerveDrivetrain.getInstance();
        superstructure = Superstructure.getInstance();

        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        // TODO: check if stop Handoff only is possible or both Handoff & Spindexer is needed
        
        // 
        if (swerve.isUnderTrench()) {
            new SuperstructureStow();
            new SpindexerStop().alongWith(new HandoffStop());
        } 
        if (swerve.isInOpponentZone()){
            new SpindexerStop().alongWith(new HandoffStop())
                .andThen(new WaitUntilCommand(superstructure::atTolerance))
                .andThen(new HandoffRun()).alongWith(new SpindexerRun());
        }

        // Prevent shooting from hard to aim places
        if (swerve.isBehindTower() || swerve.isBehindHub()) {
            new SpindexerStop().alongWith(new HandoffStop());
        }
    }
}
