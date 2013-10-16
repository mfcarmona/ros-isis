package upc.lsi.kemlg.cbr.distances;

/**
 * This interface defines the methods that should have any object to
 * be comparable through a distance criteria.<br> 
 * <br>
 * @see ComposedDistance
 * @see SimilarityMeasure
 */
public interface DistanceComparable 
{      

  //====||===============================================================
  //===\||/======= metodos a implementar por las clases =================
  //====\/===============================================================
  
  /**
   * Computes the continuous distance among the current object and the given one as argument. <br> 
   * <br>
   * {PRE: x is an instance of a class that implements DistanceComparable,
   *       d2 is an instance of a class that implements DistanceComparable,  
   * }<br>
   * {POST: returns the continuous distance among x and d2.
   * }<br>
   * <br>
   * @param d2 the object to compare with.
   * @return float the continuous distance among the current object and d2.
   */
   public float continuousDistanceTo(DistanceComparable d2);   

    
  /**
   * Computes the discrete distance among the current object and the given one as argument. <br> 
   * <br>
   * {PRE: x is an instance of a class that implements DistanceComparable,
   *       d2 is an instance of a class that implements DistanceComparable,  
   * }<br>
   * {POST: returns the discrete distace among x and d2.
   * }<br>
   * <br>
   * @param d2 the object to compare with.
   * @return float the discrete distance among the current object and d2.
   */
   public float discreteDistanceTo(DistanceComparable d2);   


  /**
   * Gets the weight of the object. <br> 
   * <br>
   * {PRE: x is an instance of a class that implements DistanceComparable  
   * }<br>
   * {POST: returns the weight of x.
   * }<br>
   * <br>
   * @return float the weight of the object.
   */
   public float getWeight();   
       
}