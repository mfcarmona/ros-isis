package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.distances.*;

/**
 * This class defines a linear attribute, an subclass of AttributeVal.
 * <br>
 * Linear attributes are the ones which can take any value in a continuous range,
 * from the minimum value to the maximum one.
 * As an extension of the AttributeVal class, all the instances of 
 * LinearAttribute have a quantitative value (the exact value that it
 * takes) and a qualitative value (the name of the category -Modality- where
 * the quantitative value belongs).
 * <b>Note: In this class the value is always represented as a float.</b> If it's wanted to be
 * used with integer values, then a conversion must be done first.
 * @see AttributeVal
 * @see AttributeDef
 * @see AttributeTable
 * @see Modality
 */
public class LinearAttribute extends AttributeVal
                             implements java.io.Serializable
{      

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** The quantitative value */
    private float quantVal;  
      


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================


  /**
   * Creates a missing linear attribute value, following the given attribute definition. <br> 
   * It checks if def is a AttributeDef that defines linear modalities. 
   * If not so, an exception is thrown.
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of LinearAttribute has been created, 
            or an exception is thrown if def is not suitable for
            a linear attribute.
   * }<br>
   * @throws java.lang.ClassCastException
   */
    public LinearAttribute (AttributeDef def)
    {
       super(def);
       
       if (!(getAttrType()==AttributeDef.LINEAR))
       {
          throw new java.lang.ClassCastException("AttributeDef does not matches with AttributeVal type");
       } 
    }
    

  /**
   * Creates a linear attribute value, following the given attribute definition and the given qualitative value but with no quantitative value. <br> 
   * It checks if def is a AttributeDef that defines linear modalities. 
   * If not so, an exception is thrown.
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of LinearAttribute has been created, 
            or an exception is thrown if def is not suitable for
            a linear attribute.
   * }<br>
   * @throws java.lang.ClassCastException
   */
    public LinearAttribute (AttributeDef def, String qualVal)
    {
       super(def,qualVal);
       
       if (!(getAttrType()==AttributeDef.LINEAR))
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       }
    }
    

  /**
   * Creates a linear attribute value with the given quantitative value and following the given attribute definition. <br> 
   * It checks if def is a AttributeDef that defines linear modalities. 
   * If not so, an exception is thrown.
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of LinearAttribute has been created, 
            or an exception is thrown if def is not suitable for
            a linear attribute.
   * }<br>
   * @throws java.lang.ClassCastException
   */
    public LinearAttribute (AttributeDef def, float quantVal)
    {
       this(def);
       setQuantVal(quantVal);
    }


  /**
   * Creates a missing linear attribute value following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * It checks if def is a AttributeDef that defines linear modalities. 
   * If not so, an exception is thrown.
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of LinearAttribute has been created, 
            or an exception is thrown if def is not suitable for
            a linear attribute.
   * }<br>
   * @throws java.lang.ClassCastException
   */
    public LinearAttribute (AttributeTable aTable, String attrName)
    {
       super(aTable, attrName);

       if (!(getAttrType()==AttributeDef.LINEAR))
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       }
        
    }
    
  /**
   * Creates a linear attribute with a given qualitative value but with no quantitative value, and following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * It checks if def is a AttributeDef that defines linear modalities. 
   * If not so, an exception is thrown.
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of LinearAttribute has been created, 
            or an exception is thrown if def is not suitable for
            a linear attribute.
   * }<br>
   * @throws java.lang.ClassCastException
   */
    public LinearAttribute (AttributeTable aTable, String attrName, String qualVal)
    {
       super(aTable,attrName,qualVal);

       if (!(getAttrType()==AttributeDef.LINEAR))
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       }

    }
    
 
  /**
   * Creates a linear attribute value with the given quantitative value and following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * It checks if def is a AttributeDef that defines linear modalities. 
   * If not so, an exception is thrown.
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of LinearAttribute has been created, 
            or an exception is thrown if def is not suitable for
            a linear attribute.
   * }<br>
   * @throws java.lang.ClassCastException
   */
    public LinearAttribute (AttributeTable aTable, String attrName, float quantVal)
    {
       this(aTable,attrName);
       setQuantVal(quantVal);
    }
    

  /**
   * Gets the quantitative value of the attribute. <br> 
   * {PRE: x is an instance of LinearAttribute. 
   * }<br>
   * {POST: returns the quantitative value of x.
   * }<br>
   * <br>
   * @return float the quantitative value of the attribute.
   */
    public float getQuantVal()
    {
       return quantVal;
    }
    

  /**
   * Sets the quantitative value of the attribute. <br> 
   * {PRE: x is an instance of LinearAttribute. 
   *       quantVal a float value to be set.
   * }<br>
   * {POST: x has quantVal as quantitative value.
   * }<br>
   * <br>
   * @param float the quantitative value for the attribute.
   */
    public void setQuantVal(float quantVal)
    {
       this.quantVal = quantVal;
       Modality mod = getAttrModality(quantVal);
       setQualVal(mod.getQualMod());
       setMissing(false);
    }
    

  /**
   * Computes the continuous distance among the current object and the given one as argument. <br> 
   * {PRE: x is an instance of LinearAttribute. 
   *       d2 is an instance of a class that implements DistanceComparable,  
   * }<br>
   * {POST: if d2 is an attribute that has the same attribute definition of x, 
               returns the continuous distance among x and d2 or throws
            otherwise an exception is thrown.
   * }<br>
   * <br>
   * @param d2 the attribute to compare with.
   * @return float the continuous distance among the current object and d2.
   * @throws java.lang.ClassCastException the given DistanceComparable attribute value
   *         doesn't follows the same attribute definition
   */
    public float continuousDistanceTo(DistanceComparable d2)
    {
       float distance=0;
       float valor_this, valor_d2;
       
       if (!(d2 instanceof LinearAttribute))
       {
          throw new java.lang.ClassCastException("Attributes are from different kind");
       }
       else if (!(getAttrName().equals(((AttributeVal) d2).getAttrName())))
       {
          throw new java.lang.ClassCastException("Attributes are different");
       }
       else
       {
          LinearAttribute la_d2 =  (LinearAttribute) d2;
         
          if (getAttrMinValue()==getAttrMaxValue())
          {
             distance=0;
          }
          else
          { 
             if (this.isMissing())
             {
                valor_this = Distance.middlePoint1D(getAttrMinValue(),getAttrMaxValue());
             }
             else
             {
                valor_this = this.getQuantVal();
             }
             
             if (la_d2.isMissing())
             {
                valor_d2 = Distance.middlePoint1D(getAttrMinValue(),getAttrMaxValue());
             }
             else
             {
                valor_d2 = la_d2.getQuantVal();
             }
             
             distance = Math.abs(valor_this - valor_d2);
             distance = distance / (getAttrMaxValue() - getAttrMinValue());
          }      
       }
       return distance;     
    }   
    
       
  /**
   * Computes the discrete distance among the current object and the given one as argument. <br> 
   * {PRE: x is an instance of LinearAttribute. 
   *       d2 is an instance of a class that implements DistanceComparable,  
   * }<br>
   * {POST: if d2 is an attribute that has the same attribute definition of x, 
               returns the discrete distance among x and d2 or throws
            otherwise an exception is thrown.
   * }<br>
   * <br>
   * @param d2 the attribute to compare with.
   * @return float the discrete distance among the current object and d2.
   * @throws java.lang.ClassCastException the given DistanceComparable attribute value
   *         doesn't follows the same attribute definition
   */
    public float discreteDistanceTo(DistanceComparable d2)
    {
      
       float distance=0;
        
       if (!(d2 instanceof LinearAttribute))
       {
          throw new java.lang.ClassCastException("Attributes are from different kind");
       }
       else if (!(getAttrName().equals(((AttributeVal) d2).getAttrName())))
       {
          throw new java.lang.ClassCastException("Attributes are different");
       }
       else
       {
          LinearAttribute la_d2 =  (LinearAttribute) d2;
          
          float meanNumMod = ((float)getAttrNumModalities() + 1.0f)/2.0f;
       
          if (getAttrNumModalities()==1)
          {
             distance = 0;
          }
          else if (this.isMissing() && la_d2.isMissing())
          {
             distance = 0;
          }
          else if (this.isMissing())
          {
             distance = Math.abs(((float) la_d2.getNumMod()) - meanNumMod);
             distance = distance / ((float)getAttrNumModalities() - 1.0f);
          }
          else if (la_d2.isMissing())
          {
             distance = Math.abs(((float)this.getNumMod()) - meanNumMod);
             distance = distance / ((float)getAttrNumModalities() - 1.0f);
          }
          else
          {
             distance = Math.abs((float)(this.getNumMod() - la_d2.getNumMod()));
             distance = distance / ((float)getAttrNumModalities() - 1.0f);
          }      
       }
       return distance;         
    }
    

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of LinearAttribute. 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString() //provisional
    {
       String res = "";
       
       res = "\n=====attribute " + getAttrName() + "=====\n";
       res += "type: " + getStrAttrType() + "\n";
       res += "weight: " + getWeight() + "\n";
       res += "value: " + getQuantVal() + " (" + getQualVal() + ")\n";
       res += "=======================================================";
       
       return res;
    }

}