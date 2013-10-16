package upc.lsi.kemlg.cbr.distances;

/**
 * This class defines the Discrete Minkowsky Distance, a kind of
 * Minkowsky Distance where qualitative values of each attribute 
 * are taken to compute the distance.
 * <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface, and whose elements
 * are subclasses of DistanceComparable.<b>
 * @see MinkowskyDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public class DiscreteMinkowsky extends MinkowskyDistance
                               implements java.io.Serializable 
{      

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================


  /** 
   * Creates a Discrete Minkowsky Weighted Distance with order = 3. <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of DiscreteMinkowsky has been created.
   * }<br>
   */
    public DiscreteMinkowsky()
    {
       super();
    }
    

  /** 
   * Creates a Discrete Minkowsky Weighted Distance with the given order. <br>
   * {PRE: order is a integer value greater than 0.
   * }<br>
   * {POST: an instance of DiscreteMinkowsky has been created.
   * }<br>
   */
    public DiscreteMinkowsky(int order)
    {
       super(order);
    }
    

  /**
   * Computes the discrete distance among two elements of different distance collections. <br>
   * To do so, the qualitative value of each element is taken.<br>
   * {PRE: x is an instance of DiscreteMinkowsky,
   *       d1 is an instance of DistanceComparable and
   *       d2 is an instance of DistanceComparable
   * }<br>
   * {POST: returns the discrete distance among d1 and d2.
   * }<br>
   * @param d1 a DistanceComparable to be compared. 
   * @param d2 a DistanceComparable to be compared. 
   * @return float the discrete distance among both elements. 
   */
    protected float distanceTwoItems (DistanceComparable d1,
                                      DistanceComparable d2)
    {
       return d1.discreteDistanceTo(d2);
    }
                                                                                        
}