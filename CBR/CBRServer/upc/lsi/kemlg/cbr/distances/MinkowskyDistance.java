package upc.lsi.kemlg.cbr.distances;

/**
 * This abstract class defines a family of distance measures named 
 * Minkowsky distances. 
 * A Minkowsky distance is a distance that gives prominency to
 * the weights assigned to each component of the elements to compare.
 * For instance, a goemetric interpretation of the role of weights 
 * in a Minkowsky distance among points p1 and p2 is depicted in the
 * following figure: 
 * <pre>
 * <br> 
 *              |                    
 *             5|  p1 x.              
 *              |          .             
 *             4|              .            
 *              |                  .           
 *             3|                      .          
 *              |                          .         
 *             2|                              .        
 *              |                                  .       
 *             1|                                  x p2     
 *              |                    
 *            0 -------------------------------------------    
 *              0      1      2      3      4      5      6  <br>
 * <br>
 * </pre>
 * <br>
 * The horizontal axis has a weight greater than 
 * the vertical one, so the Minkowsky Distance gives more influence
 * to the horizontal axis in the computation of distances.
 * <br>
 * Prominence of weights is controled by the order r of the Minkowsky
 * Distance. The full formula is the following:
 * <br>
 * d(d1,d2) = (sum[k=1..n] (wk^r * |d(c1k,c2k)|^r) / sum[k=1,n](weight^r) )^(1/r) <br>
 * <br>
 * where di is a instance of DistanceCollection, cij is the j element
 * in di (an instance of DistanceComparable), wj is the weight to 
 * apply to distance among cij components, and r is the order of the distance.
 * <br>
 * Usually the order value to be taken is 3 (Minkowsky Distance of order 3).
 * An order=2 is equivalent to an Euclidean Weighted Distance,
 * (that has an optimized implementation in class EuclideanDistance),
 * while an order= 1 is equivalent to a Manhattan Weighted Distance,
 * (that has an optimized implementation in class ManhattanDistance). 
 * <br>.
 * <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface.</b>
 * @see ComposedDistance
 * @see ManhattanDistance
 * @see EuclideanDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public abstract class MinkowskyDistance extends ComposedDistance
                                        implements java.io.Serializable 
{  
    /** The order of the Minkowsky Distance */
    int order=3;
    


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================


  /** 
   * Creates a Minkowsky Weighted Distance with order = 3. <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: true
   * }<br>
   * {POST: an instance of any non-abstract subclass of MinkowskyDistance has been created.
   * }<br>
   */
    public MinkowskyDistance()
    {
       super();
    }
    
  /** 
   * Creates a Minkowsky Weighted Distance with the given order. <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: order is a integer value greater than 0.
   * }<br>
   * {POST: an instance of any non-abstract subclass of MinkowskyDistance has been created.
   * }<br>
   */
    public MinkowskyDistance(int order)
    {
       this.order = order;
    }
      
  
  /**
   * Computes the Minkowsky Weighted Distance among two distance collections. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of MinkowskyDistance,
   *       d1 is an instance of DistanceCollection and
   *       d2 is an instance of DistanceCollection
   * }<br>
   * {POST: returns a distance among d1 and d2.
   * }<br>
   * @param d1 a DistanceCollection to be compared. 
   * @param d2 a DistanceCollection to be compared. 
   * @return float the distance among both distance collections. 
   */
    public float computeDistance(DistanceCollection d1,
                                 DistanceCollection d2)
    {
       int size_d1, size_d2;
       java.util.Iterator it_d1, it_d2;
       DistanceComparable elem_1, elem_2;  
       float res = 0;
       float currentWeight;
       float sumWeight = 0;
      
       //comprobación de que tienen la misma cantidad de items
       if (d1.size()==d2.size())
       {
          it_d1=d1.iterator(); 
          it_d2=d2.iterator();
          
          
          while(it_d1.hasNext())
          { 
             elem_1 = (DistanceComparable) it_d1.next();
             elem_2 = (DistanceComparable) it_d2.next();
             
             currentWeight = elem_1.getWeight();
             
             res = res + (float) (Math.pow(currentWeight,order) * 
                         Math.pow(distanceTwoItems(elem_1, elem_2),2));
             sumWeight = sumWeight + currentWeight;
          }
       
          res = (float)Math.pow(res,(1.0f/order)) / sumWeight;
       }
       else
       {
          System.out.println("error!!!!!!");
       }
       
       return res;
    }
    

  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  /**
   * Computes the distance among two elements of different distance collections. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of MinkowskyDistance,
   *       d1 is an instance of DistanceComparable and
   *       d2 is an instance of DistanceComparable
   * }<br>
   * {POST: returns a distance among d1 and d2.
   * }<br>
   * @param d1 a DistanceComparable to be compared. 
   * @param d2 a DistanceComparable to be compared. 
   * @return float the distance among both elements. 
   */
    protected abstract float distanceTwoItems (DistanceComparable d1,
                                               DistanceComparable d2);
                                                                                        
}