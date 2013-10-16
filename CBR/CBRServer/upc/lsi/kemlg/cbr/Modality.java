package upc.lsi.kemlg.cbr;

/**
 * This class defines a modality of an attribute definition.<br> 
 * <br>
 * The accepted range of values of the variable is splited in several 
 * modalities, When defining categorical attributes, there's a 
 * modality for each value (category) the attribute can take. 
 * <br>
 * When defining linear attributes, each modality stands for a 
 * continuous subrange of the accepted range of values and should be
 * tagged with an useful name. Modalities of a linear attribute are
 * always ordered.
 * The attribute definition where the modality belongs states if 
 * there's an order in the modalities or not.
 * @see AttributeDef
 */
public class Modality implements Comparable, java.io.Serializable
{     

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** Attribute definition where this modality belongs  */
    private AttributeDef atr;
    
  /** numeric identifier of the modality */
    private int numMod;      
    
  /** qualitative identifier of the modality */  
    private String qualMod;  
    
  /** Bottom value of the modality (for linear attributes) */  
    private float bottomVal; //valor inferior 

  /** Top value of the modality (for linear attributes) */  
    float topVal; //valor superior
    


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  
  
  /**
   * Creates a modality which belongs to the given attribute definition, with the given numeric identifier, qualitative identifier and bottom and top values. <br> 
   * <br>
   * <b>Note: this constructor is only suitable for linear attributes</b>.
   * {PRE: atr is an instance of AttributeDef,
   *       numMod is an integer value which numerically identifies the modality,
   *       qualMod is an String value which gives a qualitative value to the modality,
   *       bottomVal is a float value,
   *       topVal is a float value and bottomVal is lower than topVal.
   * }<br>
   * {POST: an instance of Modality has been created.
   * }<br>
   */
    public Modality (AttributeDef atr, int numMod, String qualMod, 
              float bottomVal, float topVal)
    {
       this.atr = atr;
       
       if (this.atr.getAttrType() == AttributeDef.LINEAR) 
       {
          this.numMod = numMod;
          this.qualMod = qualMod;
          this.bottomVal = bottomVal;
          this.topVal = topVal;
       }
       else
       {
          throw new RuntimeException ("no sense to give quantitative interval in a categorical attribute");
       }
       
    } 
    

  /**
   * Creates a modality which belongs to the given attribute definition, with the given numeric identifier, qualitative identifier. <br> 
   * <br>
   * <b>Note: this constructor is only suitable for categorical attributes</b>.
   * {PRE: atr is an instance of AttributeDef,
   *       numMod is an integer value which numerically identifies the modality,
   *       qualMod is an String value which gives a qualitative value to the modality,
   * }<br>
   * {POST: an instance of Modality has been created.
   * }<br>
   */
    public Modality (AttributeDef atr, int numMod, String qualMod)
    {
       this.atr = atr;
       
       if (this.atr.getAttrType() == AttributeDef.CATEGORICAL) 
       {
          this.numMod = numMod;
          this.qualMod = qualMod;
       }
       else
       {
          throw new RuntimeException ("quantitative interval needed in a linear attribute");
       }       
    }   

    
  /**
   * Gets the attribute definition associed to the modality. <br> 
   * {PRE: x is an instance of Modality. 
   * }<br>
   * {POST: returns the attribute definition where x belongs.
   * }<br>
   * <br>
   * @return AttributeDef the attribute definition associed to the modality.
   */
    public AttributeDef getAttributeDef()
    {
       return atr;
    }
    

  /**
   * Gets the numeric identifier of the modality. <br> 
   * {PRE: x is an instance of Modality. 
   * }<br>
   * {POST: returns the numeric identifier of x.
   * }<br>
   * <br>
   * @return int the numeric identifier of the modality.
   */
    public int getNumMod()
    {
       return numMod;
    }
    

  /**
   * Gets the qualitative identifier of the modality. <br> 
   * {PRE: x is an instance of Modality. 
   * }<br>
   * {POST: returns the qualitative identifier of x.
   * }<br>
   * <br>
   * @return int the qualitative identifier of the modality.
   */
    public String getQualMod()
    {
       return qualMod;
    }
    

  /**
   * Gets the minimum value of the modality. <br> 
   * {PRE: x is an instance of Modality. 
   * }<br>
   * {POST: returns the bottom value of x.
   * }<br>
   * <br>
   * @return float the bottom value of the modality.
   */
    public float getBottomVal()
    {
       return bottomVal;
    }
 

  /**
   * Gets the maximum value of the modality. <br> 
   * {PRE: x is an instance of Modality. 
   * }<br>
   * {POST: returns the top value of x.
   * }<br>
   * <br>
   * @return float the top value of the modality.
   */
    public float getTopVal()
    {
       return topVal;
    }
    

  /**
   * Tells if two modalities are equal <br>
   * <b>Note: this method is only useful to search modalities 
   *       inside a AttributeDef</b>, as it only considers the name 
   *       of the modality.
   * {PRE: x is an instance of Modality. 
   *       o is an Object  
   * }<br>
   * {POST: returns true if o is an instance of Modality
   *        and the name of o is the same as the name of x.
   * }<br>
   */     
    public boolean equals (Object obj) //provisional
    {
       boolean res = false;
       String s1, s2;
      
       if (obj instanceof Modality)
       {
          s1 = getQualMod().trim();
          s2 = ((Modality) obj).getQualMod().trim();
          //System.out.println("Modality.equals:[" + s1 + "]==[" + s2 + "]?");
          res = s1.equals(s2);
       }
       
       return res;
    }
    

  /**
   * Compares this Modality to another Object.<br>
   * If the Object is a Modality and belongs to an attribute 
   * definition where the modalities are ordered, then it makes the
   * comparison of the numerical identifiers of both. Otherwise, an
   * exception is thrown.
   * <br>
   * {PRE: x is an instance of Modality. 
   *       o is an Object
   * }<br>
   * {POST: if the attribute definition of x is the same of o and defines a order among
   *        modalities, returns 0 if the attribute value of o is equal to the 
   *        attribute value of x; a value less than 0 if the 
   *        attribute value of o is greater than the attribute value
   *        of x; a value greater than 0 if the attribute value of
   *        o is lower than the attribute value of x. Otherwise, an
   *        exception is thrown.
   * }<br>
   * @param o  the Object to be compared        
   * @return  the signed comparison of the attribute values of x and o. 
   * @throws  java.lang.ClassCastException the object passed as argument is not a Modality.
   * @throws  java.lang.RuntimeException the modality is not ordered or the modalities does not belong to the same attribute definition. 
   */
    public int compareTo(Object o) 
    {
       int res = 0;
       
       if (atr.isOrdered())  //si las modalidades definen un orden
       {  
       
         if (o instanceof Modality)
         {
            if ( (this.getAttributeDef()).equals(((Modality) o).getAttributeDef()) )
            {  
               res = getNumMod() - ((Modality) o).getNumMod();
            }
            else
            {
               throw new RuntimeException ("The modalities cannot be compared. They don't belong to the same attribute definition");
            }
         }
         else
         {
            throw new ClassCastException ("Object passed is not instance of Modality");  
         }
       }
       else
       {
          throw new RuntimeException ("It's not an ordered modality");
       }
       return res;
    }


  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of Modality. 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString() //provisional
    {
       return ("" + numMod+ "-" + qualMod);
    }
    

      
}