package upc.lsi.kemlg.cbr;

import java.util.*;

/**
 * This class provides a fast-accessible structure for storing and
 * retrieving attribute definitions (AttributeDef). 
 * <br>
 * Attribute definitions are indexed by name of attribute. It behaves
 * as a Hashtable thet only accepts instances of AttributeDef.
 * @see AttributeDef
 * @see Hashtable
 */
 
public class AttributeTable extends Hashtable
                            implements java.io.Serializable
{   
  
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  /**
   * Creates a void AttributeTable. <br> 
   * <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of AttributeTable has been created.
   * }<br>
   */  
    public AttributeTable()
    {
       super();
    }
 
 
  /**
   * Maps the specified name to the specified AttributeDef. <br>
   * Neither the key (the name) nor the value (the AttributeDef)
   * can be null. 
   * <br>
   * The AttributeDef can be retrieved by calling the get method with 
   * a name that is equal to the original name.
   * <br>
   * {PRE: x is an instance of AttributeTable
   * }<br>
   * {POST: if key is instance of String, value is instance of AttributeDef,
            and key matches the name inside value, then the pair (key,value)
            is added. Otherwise, an exception is thrown.
   * }<br>
   * @throws java.lang.ClassCastException either the key is not a String or the value is not an AttributeDef.
   * @throws java.lang.RuntimeException the name provided as key does not match with the name in the AttributeDef.
   */
    public Object put(Object key, Object value) throws java.lang.ClassCastException
    {
       Object res=null;
       
       if ((value instanceof AttributeDef) &&
           (key instanceof String))
       {
           if ( ((String) key).equals( ((AttributeDef) value).getAttrName() ) )
           { 
             res = super.put(key,value);
           }
           else
           {
              throw new java.lang.RuntimeException("Error attempting to add a AttributeDef with a name that not matches");
           }
       }    
       else
       {
          throw new java.lang.ClassCastException("Cannot add elem to Collection");
       }
       return res;
    }


  /**
   * Adds an AttributeDef. <br>
   * This method automatically gets the attribute name in the AttributeDef
   * as key. The value (the AttributeDef) cannot be null. 
   * <br>
   * The AttributeDef can be retrieved by calling the get method with 
   * the name of the attribute.
   * <br>
   * {PRE: x is an instance of AttributeTable
   * }<br>
   * {POST: if value is instance of AttributeDef then the value is
   *        added as a pair (name,value). Otherwise, an exception is thrown.
   * }<br>
   * @throws java.lang.ClassCastException the value is not an AttributeDef.
   */
    public boolean add(Object value) throws java.lang.ClassCastException
    {  
       Object res=null;
       String name;
       
       if (value instanceof AttributeDef)
       {
          name = ((AttributeDef) value).getAttrName();
          res = put(name,value);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot add elem to Collection");
       }
       return (res==null);
    }    

 }