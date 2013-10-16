package upc.lsi.kemlg.cbr.distances;

/** 
 * This class provides generic methods that are related to distances.<br>
 * No instances of this class are needed, as all the methods are static.
 */

public abstract class Distance implements java.io.Serializable
{
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  /**
   * Method that computes the middle point among two points in one dimension.<br>
   * {PRE: p1 is a float value and
   *       p2 is a float value
   * }<br>
   * {POST: returns the middle point among p1 and p2.
   * }<br>
   * @param p1 a 1D point. 
   * @param p2 a 1D point. 
   * @return float the 1D middle piont among p1 and p2. 
   */
   public static float middlePoint1D(float p1, float p2)
   {
      return (p1 + p2)/(2.0f);
   }
}