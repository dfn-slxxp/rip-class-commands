package com.stuypulse.robot.commands.hood;

import com.stuypulse.robot.subsystems.superstructure.hood.Hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HomingRoutine extends InstantCommand {
    private Hood hood;
    public HomingRoutine() {
        hood = Hood.getInstance();
    }

    @Override
    public void initialize() {
        hood.setState(Hood.HoodState.HOMING);
    }
}
