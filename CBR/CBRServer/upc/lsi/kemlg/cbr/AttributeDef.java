package upc.lsi.kemlg.cbr;

import java.util.*;

/**
 * This class defines an attribute definition.<br> 
 * <br>
 * An attribute definition is useful to create descriptions about the
 * attributes that will be used later by several AttributeVal objects
 * in order to check if the value they hold follows the definition.
 * <br>
 * The accepted range of values of the variable is splited in several 
 * modalities, When describing categorical attributes, there's a 
 * modality for each value (category) the attribute can take. It can also
 * stated if the modalities define an order or not. 
 * <br>
 * When describing linear attributes, each modality stands for a 
 * continuous subrange of the accepted range of values and should be
 * tagged with an useful name. Modalities of a linear attribute are
 * always ordered. 
 * @see AttributeVal
 * @see Modality
 */
 
public class AttributeDef implements java.io.Serializable
{   
  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** 
   *  Categorical type.  
   */ 
    public final static int CATEGORICAL = 0;
    
  /** 
   *  Linear type.  
   */ 
    public final static int LINEAR = 1;
    
  /** 
   *  Name of the attribute definition.  
   */ 
    private String attrName; //nombre del atributo

  /** 
   *  Type of the attribute {CATEGORICAL or LINEAR}
   */ 
    private int attrType; 

  /** 
   *  An order among the modalities is defined.
   */ 
    private boolean ordered; //hay un orden entre las modalidades


  /** 
   *  the modalities are cyclic.
   */ 
    private boolean cyclic; //hay un orden entre las modalidades

  /** 
   *  Weight of the attribute in a description. This lets to give
   *  more weight to more important attributes and less weight to
   *  the less importants.
   */ 
    private float weight;
    
    //private String unit  // unitats de mesura en la que s'expresen els valors (opcional) 
    
  /** 
   *  The list of modalities of the attribute.
   */ 
    private Vector modalities;
    
    
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
    
  /**
   * Creates an AttributeDef with the given name, type and weight. <br> 
   * <br>
   * {PRE: name is a string suitable (informative) for naming the attribute definition,
   *       type is an int with value AttributeDef.CATEGORICAL or 
   *            AttributeDef.LINEAR, 
   *       weight is the weight of the attribute
   * }<br>
   * {POST: an instance of AttributeDef has been created.
   * }<br>
   * <br>
   * @param name the name of the attribute definition
   * @param type the type of the attribute (CATEGORICAL or LINEAR)
   * @param weight the weight of the attribute. 
   */
    public AttributeDef(String name, int type, float weight)
    {
       attrName = name;
	   cyclic = false;
       if (type==CATEGORICAL)
       {
         attrType = type;
         ordered = false; //per defecte els atributs categorics no són ordenats
       }
       else if (type==LINEAR)
       {
         attrType = type;
         ordered = true; //els atributs lineals defineixen un ordre
       }
       else throw new RuntimeException("Error!!!");
         
       this.weight = weight;
       modalities = new Vector();
    } 
    
  /**
   * Creates an AttributeDef with the given name, type, weight and order value. <br> 
   * <br>
   * {PRE: name is a string suitable (informative) for naming the attribute definition,
   *       type is an int with value AttributeDef.CATEGORICAL or 
   *            AttributeDef.LINEAR, 
   *       weight is the weight of the attribute
   *       order is a boolean which tells whether the attribute modalities have an order or not.
   * }<br>
   * {POST: an instance of AttributeDef has been created.
   * }<br>
   * <br>
   * @param name the name of the attribute definition
   * @param type the type of the attribute (CATEGORICAL or LINEAR)
   * @param weight the weight of the attribute. 
   * @param order the attribute modalities have an order
   */
    public AttributeDef(String name, int type, float weight, boolean order)
    {
       attrName = name;
	   cyclic = false;
       if (type==CATEGORICAL)
       {
         attrType = type;
         ordered = order;
       }
       else if ((type==LINEAR) && order)
       {
         attrType = type;
         ordered = true; //els atributs lineals sempre defineixen un ordre
       }
       else throw new RuntimeException("Error!!!");
         
       this.weight = weight;
       modalities = new Vector();
    } 
    

  /**
   * Gets the name of the attribute definition. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns the name of x.
   * }<br>
   * <br>
   * @return String the name of the attribute definition.
   */
    public String getAttrName()
    {
       return attrName;
    }

    
  /**
   * Gets the type of the attribute. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns the type of x.
   * }<br>
   * <br>
   * @return int the type of the attribute, (either AttributeDef.LINEAR or AttributeDef.CATEGORICAL).
   */
    public int getAttrType()   
    {
       return attrType;
    }

    
  /**
   * Gets an string representation of the type of the attribute. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns a String representing the type of x.
   * }<br>
   * <br>
   * @return String the type of the attribute, (either "LINEAR" or "CATEGORICAL").
   */
    public String getStrAttrType()
    {
       String res = "";
       
       if (attrType == CATEGORICAL) 
       {
          res = "CATEGORICAL";
       }
       else
       {
          res = "LINEAR";
       }
       return res;
    }
    

  /**
   * Tells if the modalities have an order. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns true if the modalities of x have an order.
   * }<br>
   * <br>
   * @return boolean the modalities of x have an order.
   */
    public boolean isOrdered()
    {
       return ordered;
    }  
    
  /**
   * Tells if the modalities are cyclic. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns true if the modalities of x are cyclic.
   * }<br>
   * <br>
   * @return boolean the modalities of x are cyclic.
   */
    public boolean isCyclic()
    {
       return cyclic;
    }  


    /**
     * Sets if the modalities are cyclic. <br> 
     * <br>
     * {PRE: x is an instance of AttributeDef  
     * }<br>
     * {POST: sets if the modalities of x are cyclic.
     * }<br>
     * <br>
     * @param boolean the modalities of x are cyclic.
     */
      public void setCyclic(boolean b)
      {
         cyclic = b;
      }  

    
  /**
   * Gets the weight of the attribute. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns the weight of x.
   * }<br>
   * <br>
   * @return float the weight of the attribute definition.
   */
    public float getWeight()
    {
       return weight;
    }


    public void setWeight(float w)
    {
	weight = w;
    }

  /**
   * Gets the number of modalities defined in the attribute definition. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns the number of modalities of x.
   * }<br>
   * <br>
   * @return int the number of modalities.
   */
    public int getNumModalities()
    {
       return modalities.size();
    }
    

  /**
   * Gets the minimum value of the accepted range of values for this attribute. <br> 
   * <br>
   * <b>Note: this is only useful with linear attributes</b>. Using this method with 
   * categorical attributes will cause an exception.
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns the minimum acceptable value for attribute x.
   * }<br>
   * <br>
   * @return float the minimum value accepted.
   */
    public float getMinValue()
    {
       if (attrType == CATEGORICAL) 
       {
          throw new RuntimeException("no min or max values in a categorical attribute!");
       }
       return ((Modality)modalities.firstElement()).getBottomVal();
    }
    

  /**
   * Gets the maximum value of the accepted range of values for this attribute. <br> 
   * <br>
   * <b>Note: this is only useful with linear attributes</b>. Using this method with 
   * categorical attributes will cause an exception.
   * <br>
   * {PRE: x is an instance of AttributeDef  
   * }<br>
   * {POST: returns the maximum acceptable value for attribute x.
   * }<br>
   * <br>
   * @return float the maximum value accepted.
   */
    public float getMaxValue()
    {
       if (attrType == CATEGORICAL) 
       {
          throw new RuntimeException("no min or max values in a categorical attribute!");
       }
       return ((Modality)modalities.lastElement()).getTopVal();
    }
    

  /**
   * Gets the modality of the attribute definition that is numbered as the given number. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   *       numMod is an int
   * }<br>
   * {POST: returns the modality associated with the given number in x, if it exists, else an
   *        exception is thrown.
   * }<br>
   * <br>
   * @param qualMod the number of the searched modality
   * @return Modality the modality numbered as numMod.
   */
    public Modality getModality(int numMod)
    { 
       return ((Modality)modalities.get(numMod));
    }
    

  /**
   * Gets the modality of the attribute definition that is related to the given modality name. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   *       qualMod is a String value
   * }<br>
   * {POST: returns the modality in x which has qualMod as qualitative 
   *        value, if exists; else an exception is thrown.
   * }<br>
   * <br>
   * @param qualMod the name of the searched modality
   * @return Modality the modality named as qualMod.
   */
    public Modality getModality(String qualMod)
    {
      Modality aux = null;
      int pos;

      //creamos una modalidad auxiliar para buscar
      if (getAttrType()==CATEGORICAL)
      {
         aux = new Modality(this, 0, qualMod);
      }
      else
      {
         aux = new Modality(this, 0, qualMod, -1, -1);
      }
      
      pos = modalities.indexOf(aux);
      
      if (pos!=-1)
      {  
        return ((Modality) modalities.get(pos)); 
      }
      else
      { 
        return null;
      }
    }
    

  /**
   * Gets the modality of the attribute definition where the given value belongs. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   *       quantVal is a float value
   * }<br>
   * {POST: returns the modality in x where the quantVal value belongs, that is, the
            modality which subrange holds the quantVal.
   * }<br>
   * <br>
   * @param quantVal the value to look for inside the modalities
   * @return Modality the modality where the quantVal value belongs.
   */
    public Modality getModality(float quantVal)
    {
       Modality modAct = null;
       boolean trobat = false;
       
       if (getAttrType()==CATEGORICAL)
       {
          throw new RuntimeException("no sense to search a modality by quantitative value in a categorical attribute");
       }
       else
       {
          for (Iterator i=modalities.iterator();(i.hasNext()&&(!trobat));)
          {
             modAct = (Modality)i.next();
             if (quantVal < modAct.getBottomVal())
             {
                /*nos hemos pasado... es el caso de que el valor es menor que el intervalo de la
                 primera modalidad*/
                 trobat = true;
             }
             else if (quantVal <= modAct.getTopVal())
             {
                /*el valor esta en esta modalidad*/
                trobat = true;
             }
             //else buscaremos la siguiente modalidad
          }
          
          /*- si hem trobt la modalitat ja la tenim a modAct... 
            - si no la hem trobat es que el valor es major que la última modalitat...
              assignem la última modalitat, que tenim a modAct */
       }         
                 
       return modAct;
    }   
   
                    
  /**
   * Adds a modality to the attribute definition. <br> 
   * <br>
   * {PRE: x is an instance of AttributeDef  
   *       newMod is a Modality
   * }<br>
   * {POST: newMod is one of the modalities of x.
   * }<br>
   * <br>
   * @param newMod the new modality to be added
   */
    public synchronized void addModality(Modality newMod)
    {
       modalities.add(newMod.getNumMod(),newMod);
    }
    

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of AttributeDef. 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString()  //provisional
    {
       String res;
       
       res = "\n=====Definition of attribute " + getAttrName() + "=====\n";
       res += "name: " + getAttrName() + "\n";   
       res += "type: " + getStrAttrType() + "\n";
       res += "weight: " + getWeight() + "\n";
	   res += "cyclic: " + isCyclic() + "\n";
       res += "modalities: " + modalities.toString() + "\n";
       res += "=======================================================";
       
       return res;
    }
    
        
}
