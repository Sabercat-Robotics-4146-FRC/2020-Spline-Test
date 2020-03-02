package frc.robot.Subsystems;

public class PIDController {

    // setting PID values                           kP    kI  kD        kF         kIZ peakOutput
    public final static Gains velocity =    new Gains(0.075, 0.000, 0, 0.2 * 1023 / 4000, 0, 1);
    public final static Gains turning =     new Gains( 1.5, 0.0,  4.0, 0.0, 10,  1.00 );
    //public final static Gains turning =     new Gains( 2.0, 0.0,  4.0, 0.0, 10,  1.00 );

    public final static int PIDPrimary = 0;
    public final static int PIDTurn = 1;

    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;

}