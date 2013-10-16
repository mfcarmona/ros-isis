package upc.lsi.kemlg.cbr.distances;

/**
 * This class defines the Continuous Euclidean Distance, a kind of
 * Euclidean Distance where quantitative values of each attribute 
 * are taken to compute the distance.
 * <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface, and whose elements
 * are subclasses of DistanceComparable.<b>
 * @see EuclideanDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public class ContinuousEuclidean extends EuclideanDistance 
                                 implements java.io.Serializable
{      

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  /**
   * Computes the continuous distance among two elements of different distance collections. <br>
   * To do so, the quantitative value of each element is taken.<br>
   * {PRE: x is an instance of ContinuousEuclidean,
   *       d1 is an instance of DistanceComparable and
   *       d2 is an instance of DistanceComparable
   * }<br>
   * {POST: returns the continuous distance among d1 and d2.
   * }<br>
   * @param d1 a DistanceComparable to be compared. 
   * @param d2 a DistanceComparable to be compared. 
   * @return float the continuous distance among both elements. 
   */
    protected float distanceTwoItems (DistanceComparable d1,
                                      DistanceComparable d2)
    {
       return d1.continuousDistanceTo(d2);
    }
                                                                                        
}