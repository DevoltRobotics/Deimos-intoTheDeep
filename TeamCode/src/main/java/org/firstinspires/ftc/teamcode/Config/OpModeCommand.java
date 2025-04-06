package org.firstinspires.ftc.teamcode.Config;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

public abstract class OpModeCommand extends OpMode {

    public Follower follower;

    public PedroSubsystem pedroSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public ClawSubsystem clawSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ArmSubsystem armSubsystem;
    public WristSubsystem wristSubsystem;

    //reinicia la lista de comandos
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    //corre el scheduler
    public void run() {
        CommandScheduler.getInstance().run();
    }

    //programa comandos al scheduler
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    //registra subsistemas al scheduler
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        register(
                pedroSubsystem = new PedroSubsystem(follower),
                extendoSubsystem = new ExtendoSubsystem(hardwareMap),
                clawSubsystem = new ClawSubsystem(hardwareMap),
                elevatorSubsystem = new ElevatorSubsystem(hardwareMap),
                intakeSubsystem = new IntakeSubsystem(hardwareMap),
                armSubsystem = new ArmSubsystem(hardwareMap),
                wristSubsystem = new WristSubsystem(hardwareMap)
        );

        initialize();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        run();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();

}