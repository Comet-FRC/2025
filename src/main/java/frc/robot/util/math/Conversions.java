package frc.robot.util.math;



public class Conversions {

   

  /**

   * @param wheelRPS Wheel Velocity: (in Rotations per Second)

   * @param circumference Wheel Circumference: (in Meters)

   * @return Wheel Velocity: (in Meters per Second)

   */

  public static double RPSToMPS(double wheelRPS, double circumference){

    double wheelMPS = wheelRPS * circumference;

    return wheelMPS;

  }



  /**

   * @param wheelMPS Wheel Velocity: (in Meters per Second)

   * @param circumference Wheel Circumference: (in Meters)

   * @return Wheel Velocity: (in Rotations per Second)

   */

  public static double MPSToRPS(double wheelMPS, double circumference){

    double wheelRPS = wheelMPS / circumference;

    return wheelRPS;

  }



  /**

   * @param wheelRotations Wheel Position: (in Rotations)

   * @param circumference Wheel Circumference: (in Meters)

   * @return Wheel Distance: (in Meters)

   */

  public static double rotationsToMeters(double wheelRotations, double circumference){

    double wheelMeters = wheelRotations * circumference;

    return wheelMeters;

  }



  /**

   * @param wheelMeters Wheel Distance: (in Meters)

   * @param circumference Wheel Circumference: (in Meters)

   * @return Wheel Position: (in Rotations)

   */

  public static double metersToRotations(double wheelMeters, double circumference){

    double wheelRotations = wheelMeters / circumference;

    return wheelRotations;

  }


  /**
     * @param degrees Mechanism Position: (in Degrees)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor Rotation: (in Rotations)
     */
    public static double degreesToTalon(double mechDeg, double gearRatio) {
      double motorDeg = mechDeg * gearRatio;
      double motorRotations = motorDeg / 360.0;
      return motorRotations;
  } 

}