package upc.lsi.kemlg.cbr.distances;

/**
 * This abstract class defines a kind of distance measures named 
 * Manhattan distances. 
 * A Manhattan distance is the shortest path among two points,
 * with the restriction that the path segments should always be 
 * parallel to one of the coordinate axis. <br>
 * For instance, a goemetric interpretation of the Manhattan 
 * distance among points p1 and p2 is depicted in the 
 * following figure:
 * <pre>
 * <br>
 *                    |                    
 *                   5|  p1 x.........     
 *                    |              .     
 *                   4|              .     
 *                    |              .     
 *                   3|              .     
 *                    |              .     
 *                   2|              .     
 *                    |              .     
 *                   1|              x     
 *                    |                    
 *                   0 ------------------    
 *                    0  1  2  3  4  5  6  
 * <br>
 *             d(p1,p2) = (5-2) + (5-1) = 7  <br>
 * <br>
 * </pre>
 * This class implements a weighted version of the Manhattan Distance.
 * The full formula is the following: 
 * d(d1,d2) = sum[k=1..n](wk * |d(c1k,c2k)|) / sum[k=1,n](weight) <br>
 * <br>
 * where di is a instance of DistanceCollection, cij is the j element
 * in di (an instance of DistanceComparable) and wj is the weight to 
 * apply to distance among cij components.
 * <br>
* <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface.</b>
 * @see ComposedDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public abstract class ManhattanDistance extends ComposedDistance
                                        implements java.io.Serializable 
{      

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  
  /**
   * Computes the Manhattan Weighted Distance among two distance collections. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of ManhattanDistance,
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
             res = res + currentWeight * distanceTwoItems(elem_1, elem_2);
             sumWeight = sumWeight + currentWeight;
          }
       
          res = res / sumWeight;
       }
       else
       {
		 	System.out.println("ManhattanDistanceClass Error!!!!!!");
			System.out.println("Elementos de distinto tamaño.");
			System.out.println("Compruebe que la base de datos escogida es correcta.");
       }
       
       return res;
    }
    


  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  /**
   * Computes the distance among two elements of different distance collections. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of ManhattanDistance,
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