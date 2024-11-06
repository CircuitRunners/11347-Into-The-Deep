package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.armcommands.armTestCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.Claw

class ArmToScoringCommandAuto(arm: Arm, claw: Claw, preset: Presets) : ParallelCommandGroup() {
    enum class Presets {
        HOLD,
        REST,
        HOVER_SUB,
        GRAB_SUB,
        AUTO,
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
                    Presets.HOLD ->
                        armTestCommand(arm, arm.armPosition, true)
                    Presets.REST ->
                        armTestCommand(arm, ArmPositions.REST.position, true)
                    Presets.HOVER_SUB ->
                        armTestCommand(arm, ArmPositions.HOVER_SUB.position, true, true)
                    Presets.GRAB_SUB ->
                        armTestCommand(arm, ArmPositions.GRAB_SUB.position, true, true)
                    Presets.BASKET_HIGH ->
                        armTestCommand(arm, ArmPositions.BASKET_HIGH.position, true, false)
                    Presets.MID ->
                        armTestCommand(arm, ArmPositions.MID.position, true)
                    Presets.AUTO ->
                        armTestCommand(arm, ArmPositions.AUTO.position, true,true)

                }
            )
        )
        addRequirements(arm, claw)
    }
}
