package org.firstinspires.ftc.teamcode.commands.presets

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.armcommands.ProfiledArmCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Slides

class ArmToScoringCommand(arm: ArmCorrected, claw: Claw, preset: Presets) : ParallelCommandGroup() {
    enum class Presets {
//        HOLD,
        REST,
        HOVER_SUB,
        GRAB_SUB,
        SPECIMEN,
        TOPBAR,
        AUTO,
        BASKET_HIGH,
        MID
    }

    init {
        addCommands(

            when (preset) {
//                Presets.HOLD ->
//                    ProfiledArmCommand(arm, arm.armPosition, true)
                Presets.REST ->
                    ProfiledArmCommand(arm, ArmPositions.REST.position, true, false, true, 3500)
                Presets.HOVER_SUB ->
                    ProfiledArmCommand(arm, ArmPositions.HOVER_SUB.position, true, true, false)
                Presets.GRAB_SUB ->
                    ProfiledArmCommand(arm, ArmPositions.GRAB_SUB.position, true, true, false)
                Presets.SPECIMEN ->
                    ProfiledArmCommand(arm, ArmPositions.SPECIMEN.position, true,true, false)
                Presets.BASKET_HIGH ->
                    ProfiledArmCommand(arm, ArmPositions.BASKET_HIGH.position, true, false, false)
                Presets.MID ->
                    ProfiledArmCommand(arm, ArmPositions.MID.position, true)
                Presets.AUTO ->
                    ProfiledArmCommand(arm, ArmPositions.AUTO.position, true,true)
                Presets.TOPBAR ->
                    ProfiledArmCommand(arm, ArmPositions.TOPBAR.position, true)
            }
        )
        addRequirements(arm, claw)
    }
}
