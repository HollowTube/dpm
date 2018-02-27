package ca.mcgill.ecse211.Localization;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
