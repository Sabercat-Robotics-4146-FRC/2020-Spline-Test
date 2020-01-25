package frc.robot.Auto;

public class Timer {
	private long startTime = 0;
	private long thisTime = 0;
	private long lastTime = 0;
	private double dt = 0;
	
	public Timer() {
		startTime = System.nanoTime();
		thisTime = startTime;
		lastTime = startTime;
	}
	
	public void reset() {
		startTime = System.nanoTime();
		thisTime = startTime;
		lastTime = startTime;
	}
	
	public void update() {
		thisTime = System.nanoTime();
		dt = convertNanoToSec(thisTime - lastTime);
		lastTime = thisTime;
	}
	public double getDT() {
		return dt;
	}
	
	public double convertNanoToSec(long nano) {		
			return (nano * (1e-9));
	}
	
	public double timeSinceStart() {
		return convertNanoToSec( System.nanoTime() - startTime );
	}
	/*public void reset_start() {  // If you can't find any reason to have this, take it out.
		startTime = System.nanoTime();
	}*/
	public static void waitMilli() {
		try {
			Thread.sleep(1);
		} catch (InterruptedException e) {
			// MOVE THIS TO ITERATIVE_TIMER
			//OK
			System.out.println("Thread.sleep was Interrupted!");
			e.printStackTrace();
		}
	}
	
	public static void waitTime(int i) {
		try {
			Thread.sleep(i);
		} catch (InterruptedException e) {
			System.out.println("Thread.sleep was Interrupted!");
			e.printStackTrace();
		}
	}
	
}