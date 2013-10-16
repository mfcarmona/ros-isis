package upc.lsi.kemlg.cbr.distances;

/**
 * This class defines the Continuous Minkowsky Distance, a kind of
 * Minkowsky Distance where quantitative values of each attribute 
 * are taken to compute the distance.
 * <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface, and whose elements
 * are subclasses of DistanceComparable.<b>
 * @see MinkowskyDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public class ContinuousMinkowsky extends MinkowskyDistance
                                 implements java.io.Serializable 
{      

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================


  /** 
   * Creates a Continuous Minkowsky Weighted Distance with order = 3. <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of ContinuousMinkowsky has been created.
   * }<br>
   */
    public ContinuousMinkowsky()
    {
       super();
    }
    

  /** 
   * Creates a Continuous Minkowsky Weighted Distance with the given order. <br>
   * {PRE: order is a integer value greater than 0.
   * }<br>
   * {POST: an instance of ContinuousMinkowsky has been created.
   * }<br>
   */
    public ContinuousMinkowsky(int order)
    {
       super(order);
    }
    

  /**
   * Computes the continuous distance among two elements of different distance collections. <br>
   * To do so, the quantitative value of each element is taken.<br>
   * {PRE: x is an instance of ContinuousMinkowsky,
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