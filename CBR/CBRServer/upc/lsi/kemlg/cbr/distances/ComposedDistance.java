package upc.lsi.kemlg.cbr.distances;

import upc.lsi.kemlg.cbr.*;

/**
 * This abstract class defines a family of similarity measures where 
 * a composition of distances among sets of elements are chosen as 
 * similarity criteria among case descriptions. <br>
 * @see CaseDescription
 * @see SimilarityMeasure
 */
public abstract class ComposedDistance extends SimilarityMeasure
                                       implements java.io.Serializable
{      

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  

  /**
   * Computes the similarity ratio among two case descriptions. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of ComposedDistance,
   *       d1 is an instance of CaseDescription and
   *       d2 is an instance of CaseDescription
   * }<br>
   * {POST: returns a similarity ratio among d1 and d2.
   * }<br>
   * @param d1 a case description to be compared. 
   * @param d2 a case description to be compared. 
   * @return float the similarity ratio among both case descriptions. 
   */
   public float computeSimilarity(CaseDescription d1,
                                  CaseDescription d2)
                                  throws java.lang.ClassCastException
   {
      float res = -1;
      
      if ((d1 instanceof DistanceCollection) &&
          (d2 instanceof DistanceCollection))
      { 
         res =  computeDistance((DistanceCollection) d1, 
                                (DistanceCollection) d2);
      }
      else
      {
         throw new java.lang.ClassCastException("Cannot compute distance among this two cases. One of the Case Descriptions is not a DistanceCollection. ");
      }

      return res; 
   }
    


  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  /**
   * Computes the distance among two case descriptions. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of ComposedDistance,
   *       d1 is an instance of CaseDescription and
   *       d2 is an instance of CaseDescription
   * }<br>
   * {POST: returns a distance among d1 and d2.
   * }<br>
   * @param d1 a case description to be compared. 
   * @param d2 a case description to be compared. 
   * @return float the distance among both case descriptions. 
   */
   public abstract float computeDistance(DistanceCollection d1,
                                         DistanceCollection d2);   
                                         
}