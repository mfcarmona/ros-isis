package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.distances.*;

/**
 * This class defines a categorical attribute, an subclass of AttributeVal.
 * <br>
 * Categorical attributes are the ones which can take a discrete
 * number of values (categories or modalities). 
 * <br>
 * The modalities an instance of this class can take are defined in
 * the attribute definition (AttributeDef) associed to the 
 * categorical attribute. 
 * As an extension of the AttributeVal class, all the instances of 
 * CategoricalAttribute have a qualitative value (the name of the 
 * Modality).
 * @see AttributeVal
 * @see AttributeDef
 * @see AttributeTable
 * @see Modality
 */
public class CategoricalAttribute extends AttributeVal
                                  implements java.io.Serializable
{      
      
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  
  /**
   * Creates a missing categorical attribute value, following the given attribute definition. <br> 
   * <br>
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of CategoricalAttribute has been created.
   * }<br>
   */
    public CategoricalAttribute (AttributeDef def)
    {
       super(def);
       
       if (getAttrType()!=AttributeDef.CATEGORICAL)
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       } 
    }
    

  /**
   * Creates a categorical attribute with the given qualitative value, following the given attribute definition. <br> 
   * {PRE: def is an instance of AttributeDef.
   *       qualVal is an String value that matches the name of one of the
   *               modalities inside def 
   * }<br>
   * {POST: an instance of CategoricalAttribute has been created.
   * }<br>
   */
    public CategoricalAttribute (AttributeDef def, String qualVal)
    {
       super(def,qualVal);
       
       if (getAttrType()!=AttributeDef.CATEGORICAL)
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       }
    }

    
  /**
   * Creates a missing categorical attribute value, following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * <br>
   * {PRE: aTable is an instance of AttributeTable,
   *       attrName is a String value that matches the name of an attribute 
                    definition inside aTable.
   * }<br>
   * {POST: an instance of CategoricalAttribute has been created.
   * }<br>
   */
    public CategoricalAttribute (AttributeTable aTable, String attrName)
    {
       super(aTable, attrName);

       if (getAttrType()!=AttributeDef.CATEGORICAL)
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       }
        
    }
    

  /**
   * Creates a categorical attribute with the given qualitative value, following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * {PRE: aTable is an instance of AttributeTable,
   *       attrName is a String value that matches the name of an attribute 
   *               definition inside aTable.
   *       qualVal is an String value that matches the name of one of the
   *               modalities inside the attribute definition named attrName
   * }<br>
   * {POST: an instance of CategoricalAttribute has been created.
   * }<br>
   */
    public CategoricalAttribute (AttributeTable aTable, String attrName, String qualVal)
    {
       super(aTable,attrName,qualVal);

       if (getAttrType()!=AttributeDef.CATEGORICAL)
       {
          throw new java.lang.ClassCastException("AttributeDef dos not matches with AttributeVal type");
       }

    }
    
    
  /**
   * Computes the continuous distance among the current object and the given one as argument. <br> 
   * As categorical attributes have only discrete values, this method
   * really computes a discrete distance. 
   * {PRE: x is an instance of CategoricalAttribute. 
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
       return discreteDistanceTo(d2);     
    }   
    
    
    
  /**
   * Computes the discrete distance among the current object and the given one as argument. <br> 
   * {PRE: x is an instance of CategoricalAttribute. 
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
        
       if (!(d2 instanceof CategoricalAttribute))
       {
          throw new java.lang.ClassCastException("Attributes are from different kind");
       }
       else if (!(getAttrName().equals(((AttributeVal) d2).getAttrName())))
       {
          throw new java.lang.ClassCastException("Attributes are different");
       }
       else
       {
          CategoricalAttribute ca_d2 =  (CategoricalAttribute) d2;
          
          if (this.isMissing() && ca_d2.isMissing())
          {
             distance = 0;
          }
          else if (this.isMissing() || ca_d2.isMissing())
          {
             distance = ((float)getAttrNumModalities() - 1.0f) 
                        / (float)getAttrNumModalities();
          }
          else 
          {
             if(this.getNumMod()==ca_d2.getNumMod())
             {
                distance = 0;
             }
             else
             {
                distance = 1;
             }   
          }     
       }
       return distance;         
    }
    

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of CategoricalAttribute. 
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
       res += "value: " + getQualVal() + "\n";
       res += "=======================================================";
       
       return res;
    }
       
}