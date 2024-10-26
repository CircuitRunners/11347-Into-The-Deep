package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.armcommands.ProfiledArmCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.Claw

class ArmToScoringCommand(arm: Arm, claw: Claw, preset: Presets) : ParallelCommandGroup() {
    enum class Presets {
        REST,
        HOVER_SUB,
        GRAB_SUB,
        BASKET_HIGH,
        MID
    }

    init {
        addCommands(
                ParallelCommandGroup(
                        SequentialCommandGroup(
                                WaitCommand(800),
                                InstantCommand({

                                }),
                        ),
                        InstantCommand({
                            claw.close()
                        }),
                        when (preset) {
                            Presets.REST ->
                                ProfiledArmCommand(arm, ArmPositions.REST.position, true)
                            Presets.HOVER_SUB ->
                                ProfiledArmCommand(arm, ArmPositions.HOVER_SUB.position, false)
                            Presets.GRAB_SUB ->
                                ProfiledArmCommand(arm, ArmPositions.GRAB_SUB.position, false, true)
                            Presets.BASKET_HIGH ->
                                ProfiledArmCommand(arm, ArmPositions.BASKET_HIGH.position, false)
                            Presets.MID ->
                                ProfiledArmCommand(arm, ArmPositions.MID.position, true)

                        }
                )
        )
        addRequirements(arm)
    }
}
