package upc.lsi.kemlg.cbr;

/**
 * This abstract class defines the standard to be followed by all 
 * measure criterias to be used to compute similarities among
 * case descriptions. 
 * @see CaseDescription
 */
public abstract class SimilarityMeasure implements java.io.Serializable
{      

  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  /**
   * Computes the similarity ratio among two case descriptions. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of SimilarityMeasure,
   *       d1 is an instance of CaseDescription and
   *       d2 is an instance of CaseDescription
   * }<br>
   * {POST: returns a similarity ratio among d1 and d2.
   * }<br>
   * @param d1 a case description to be compared. 
   * @param d2 a case description to be compared. 
   * @return float the similatity ratio among both case descriptions. 
   */
    public abstract float computeSimilarity(CaseDescription d1,
                                            CaseDescription d2);   
}