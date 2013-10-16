package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.distances.*;


/**
 * This class defines fields and methods useful for all attribute values.<br> 
 * <br>
 * All attribute values are instances of one of the subclasses of this class, 
 * as it provides a connection with the attribute definition (AttributeDef)
 * and also defines the procedures to compute distances.
 * <br>
 * This class provides the fields and methods common to all the kinds of
 * attributes, which are subclasses of this class. For instance, all kinds of
 * attributes should have a qualitative value. In categorical attributes
 * (CategoticalAttribute), the qualitative value is the category name 
 * (Modality). In linear attributes (LinearAttribute) it corresponds
 * to the modality where the numerical value belongs.
 * <br>
 * To create an attribute value, either a reference to the attribute definition
 * (AttributeDef) or a reference to an AttributeTable and the name of the attribute
 * (in order to find the proper AttributeDef) should be given.
 * @see AttributeDef
 * @see AttributeTable
 * @see Modality
 * @see LinearAttribute
 * @see CategoricalAttribute
 */
public abstract class AttributeVal implements DistanceComparable, Comparable,
                                           java.io.Serializable 
{      

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================
  
  /** 
   * The name of the attribute 
   */
    private String attrName;   //nombre del atributo


  /** 
   * Qualitative value 
   */
    private String  qualVal;   //valor cualitativo


  /** 
   * The modality number where it belongs 
   */
    private int nMod = -1;     //número de modalidad  


  /** 
   * The attribute definition
   */
    private AttributeDef def;  //definición del atributo


  /** 
   * The attribute is a missing val 
   */
    private boolean missingVal = true;
      
  
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  
  /**
   * Creates a missing attribute value, following the given attribute definition. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: def is an instance of AttributeDef.
   * }<br>
   * {POST: an instance of any non-abstract subclass of AttributeVal has been created.
   * }<br>
   */
    public AttributeVal (AttributeDef def)
    {
       attrName = def.getAttrName();
       this.def = def;
    }
   
    
  /**
   * Creates an attribute with the given qualitative value, following the given attribute definition. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: def is an instance of AttributeDef.
   *       qualVal is an String value that matches the name of one of the
   *               modalities inside def 
   * }<br>
   * {POST: an instance of any non-abstract subclass of AttributeVal has been created.
   * }<br>
   */
    public AttributeVal (AttributeDef def, String qualVal)
    {
       this(def);
       setQualVal(qualVal);
       missingVal = false;
    }

    
  /**
   * Creates a missing attribute value, following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: aTable is an instance of AttributeTable,
   *       attrName is a String value that matches the name of an attribute 
                    definition inside aTable.
   * }<br>
   * {POST: an instance of any non-abstract subclass of AttributeVal has been created.
   * }<br>
   */
    public AttributeVal (AttributeTable aTable, String attrName)
    {
       this.attrName=attrName;
       def = (AttributeDef)aTable.get(attrName); 
       if (def == null)
       {
          throw new RuntimeException("Error: l'atribut no existeix a la taula");
       } 
    }

  
  /**
   * Creates an attribute with the given qualitative value, following the attribute definition that can be found inside the given attribute table looking for the given attribute name. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: aTable is an instance of AttributeTable,
   *       attrName is a String value that matches the name of an attribute 
   *               definition inside aTable.
   *       qualVal is an String value that matches the name of one of the
   *               modalities inside the attribute definition named attrName
   * }<br>
   * {POST: an instance of any non-abstract subclass of AttributeVal has been created.
   * }<br>
   */
    public AttributeVal (AttributeTable aTable, String attrName, String qualVal)
    {   
       this(aTable,attrName);
       setQualVal(qualVal);
       missingVal = false;
    }
  
             
  /**
   * Gets the name of the attribute. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the name of x.
   * }<br>
   * <br>
   * @return String the name of the attribute.
   */
    public String getAttrName()
    {
       return attrName;
    }
  
  
  /**
   * Gets the qualitative value of the attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the qualitative value of x.
   * }<br>
   * <br>
   * @return String the qualitative value of the attribute.
   */
    public String getQualVal()
    {
       return qualVal;
    }
  
  
  /**
   * Sets the qualitative value of the attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       value is a String value that matches one of the modalities 
   *             of the attribute definition of x  
   * }<br>
   * {POST: value is the new qualitative value of x, or an exception is 
            thrown.
   * }<br>
   * <br>
   * @param value the qualitative value of the attribute.
   * @throws java.lang.RuntimeException value is not one of the modalities
   *         associed to this attribute.
   */
    public void setQualVal(String value)
    {
       Modality modAux = def.getModality(value);     
       
       if (modAux == null)
       {
          throw new RuntimeException("Error: no existeix la modalitat");
       }
       else
       {
          qualVal=value;
          nMod=modAux.getNumMod();
       }
       
       missingVal=false;
    }
  
  
  /**
   * Tells if the attribute value is a missing value. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns true if the attribute has a missing value.
   * }<br>
   * <br>
   * @return boolean the attribute's value is missing.
   */
    public boolean isMissing()
    {
       return missingVal;
    }
       
    
  /**
   * Gets the modality number of the attribute definition where this attribute belongs. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the modality number in the attribute definition of x where 
   *        the value of x belongs.
   * }<br>
   * <br>
   * @return int the modality number where the attribute value belongs.
   */
    public int getNumMod()
    {
       return nMod;
    }


  /**
   * Gets the attribute definition associed to the attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the attribute definition of x.
   * }<br>
   * <br>
   * @return AttributeDef the attribute definition of the attribute.
   */
    public AttributeDef getAttributeDef()
    {
       return def;
    }


    //------          métodos de comparación de atributos.         ------
    //------ (sólo son útiles para la búsqueda de atributos dentro ------
    //------  de los casos... por ello solo toman en cuenta el     ------
    //------                 nombre del atributo)                  ------


  /**
   * Tells if two attributes are equal <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * <b>Also note: this method is only useful to search attribute values 
   *       inside a Case Description</b>, as it only considers the name 
   *       of the attribute.
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       o is an Object  
   * }<br>
   * {POST: returns true if o is an instance of any non-abstract subclass
   *        of AttributeVal and the name of o is the same as the name of x.
   * }<br>
   */     
    public boolean equals(Object o)
    {
       boolean res = false;
       
       if (o instanceof AttributeVal)
       {
          res = getAttrName().equals(((AttributeVal) o).getAttrName());
       }
       // else res es false, como ya esta desde que se declaró
       
       return res;
    }
    
    
    //------- métodos de AttributeDef que los AttributeVal ofrecen ------
    //-------            (delegando al AttributeDef)               ------
  
  /**
   * Gets the number of modalities defined in the attribute definition. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the number of modalities of x.
   * }<br>
   * <br>
   * @return int the number of modalities.
   */
    public int getAttrNumModalities()
    {
       return def.getNumModalities();
    }

  
  /**
   * Gets the minimum value of the accepted range of values for this attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * <b>Also note: this is only useful with linear attributes</b>. Using this method with 
   * categorical attributes will cause an exception.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the minimum acceptable value for attribute x.
   * }<br>
   * <br>
   * @return float the minimum value accepted.
   */
    public float getAttrMinValue()
    {
       return def.getMinValue();
    }
    

  /**
   * Gets the maximum value of the accepted range of values for this attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * <b>Also note: this is only useful with linear attributes</b>. Using this method with 
   * categorical attributes will cause an exception.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the maximum acceptable value for attribute x.
   * }<br>
   * <br>
   * @return float the maximum value accepted.
   */
    public float getAttrMaxValue()
    {
       return def.getMaxValue();
    }
    

  /**
   * Gets the type of the attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the type of x.
   * }<br>
   * <br>
   * @return int the type of the attribute, (either AttributeDef.LINEAR or AttributeDef.CATEGORICAL).
   */
    public int getAttrType()
    {  
       return def.getAttrType();
    }
    

  /**
   * Gets an string representation of the type of the attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns a String representing the type of x.
   * }<br>
   * <br>
   * @return String the type of the attribute, (either "LINEAR" or "CATEGORICAL").
   */
    public String getStrAttrType()
    {
       return def.getStrAttrType();
    }

    
  /**
   * Gets the modality of the attribute definition where this attribute belongs. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the modality in the attribute definition of x where 
   *        the value of x belongs.
   * }<br>
   * <br>
   * @return Modality the modality where the attribute value belongs.
   */
    public Modality getModality()
    {
       return getAttrModality(nMod);
    }


  /**
   * Gets the modality of the attribute that is numbered as the given number. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       numMod is an int
   * }<br>
   * {POST: returns the modality associated with the given number in x, if it exists, else an
   *        exception is thrown.
   * }<br>
   * <br>
   * @param qualMod the number of the searched modality
   * @return Modality the modality numbered as numMod.
   */
    public Modality getAttrModality(int numMod)
    { 
       return def.getModality(numMod);
    }
    
    
  /**
   * Tells if the modalities have an order. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns true if the modalities of x have an order.
   * }<br>
   * <br>
   * @return boolean the modalities of x have an order.
   */
    public boolean isAttrOrdered()
    {
       return def.isOrdered();
    }

    
  /**
   * Gets the weight of the attribute. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns the weight of x.
   * }<br>
   * <br>
   * @return float the weight of the attribute.
   */
    public float getWeight()
    { 
       return def.getWeight();
    }   
    

  /**
   * Gets the modality of the attribute that is related to the given modality name. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       qualMod is a String value
   * }<br>
   * {POST: returns the modality in x which has qualMod as qualitative 
   *        value, if exists; else an exception is thrown.
   * }<br>
   * <br>
   * @param qualMod the name of the searched modality
   * @return Modality the modality named as qualMod.
   */
    public Modality getAttrModality(String qualVal)
    {
       return def.getModality(qualVal);
    }
    

  /**
   * Gets the modality of the attribute where the given value belongs. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       quantVal is a float value
   * }<br>
   * {POST: returns the modality in x where the quantVal value belongs, that is, the
            modality which subrange holds the quantVal.
   * }<br>
   * <br>
   * @param quantVal the value to look for inside the modalities
   * @return Modality the modality where the quantVal value belongs.
   */
    public Modality getAttrModality(float quantVal)
    {
       return def.getModality(quantVal);
    }


    //-------------- método de la interficie Comparable -----------------

  /**
   * Compares this AttributeVal to another Object.<br>
   * If the Object is an AttributeVal, then it makes the comparison of
   * the values of both. Otherwise, an exception is thrown.
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       o is an Object
   * }<br>
   * {POST: returns 0 if the attribute value of o is equal to the 
   *        attribute value of x; a value less than 0 if the 
   *        attribute value of o is greater than the attribute value
   *        of x; a value greater than 0 if the attribute value of
   *        o is lower than the attribute value of x.
   * }<br>
   * @param o  the Object to be compared        
   * @return  the signed comparison of the attribute values of x and o. 
   */
    public int compareTo(Object o) 
    {
       int res = 0;
       
       if (o instanceof AttributeVal)
       {
          res = getAttrName().compareTo(((AttributeVal) o).getAttrName());
       }
       else
       {
          throw new ClassCastException ("Object passed is not instance of AttributeVal");  
       }
       
       return res;
    }
        


  //====||===============================================================
  //===\||/=============== metodos auxiliares ==========================
  //====\/===============================================================
    
  /**
   * Sets the missing tag of the attribute. <br> 
   * The missing tag is used by the methods of the class to know if
   * the attribute value is a missing value.
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       missingVal is a boolean value  
   * }<br>
   * {POST: the missing tag is set to missingVal, or a exception is thrown.
   * }<br>
   * <br>
   * @param boolean the attribute's value is missing.
   * @throws java.lang.RuntimeException the missing tag cannot be tset to true, as a value is already assigned to it.
   */
    protected void setMissing(boolean missingVal)
    {
       if (missingVal && (getNumMod()!=-1))
       {
          throw new RuntimeException ("AttributeVal not missing, there's a value");
       }
       else
       {
          this.missingVal = missingVal;
       }
    }



  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  //------- métodos definidos en la interficie DistanceComparable -------
    
  /**
   * Computes the continuous distance among the current object and the given one as argument. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       d2 is an instance of a class that implements DistanceComparable,  
   * }<br>
   * {POST: returns the continuous distance among x and d2.
   * }<br>
   * <br>
   * @param d2 the object to compare with.
   * @return float the continuous distance among the current object and d2.
   */
    public abstract float continuousDistanceTo(DistanceComparable d2);   
    

  /**
   * Computes the discrete distance among the current object and the given one as argument. <br> 
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   *       d2 is an instance of a class that implements DistanceComparable,  
   * }<br>
   * {POST: returns the discrete distance among x and d2.
   * }<br>
   * <br>
   * @param d2 the object to compare with.
   * @return float the discrete distance among the current object and d2.
   */
    public abstract float discreteDistanceTo(DistanceComparable d2);
    

  /**
   * Returns a string representation of the object. <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeVal. 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public abstract String toString();    

       
}